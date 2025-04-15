// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <exception>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"
#include "auto_aim_interfaces/msg/trajectory.hpp"


namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  current_yaw = 0.0;
  up_or_down = 0.05;
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  task_pub_ = this->create_publisher<std_msgs::msg::String>("/task_mode", 10);
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  aim_time_info_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::TimeInfo>("/time_info/aim", 10);
  buff_time_info_pub_ =
    this->create_publisher<buff_interfaces::msg::TimeInfo>("/time_info/buff", 10);
  record_controller_pub_ = this->create_publisher<std_msgs::msg::String>("/record_controller", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  // Target change service cilent
  change_target_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/change");

  /*############################################################*/
  // create ROS Publisher
  robot_state_info_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStateInfo>("serial/robot_state_info", 10);
  robot_motion_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("serial/robot_motion", 10);

  event_data_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::EventData>("referee/event_data", 10);
  all_robot_hp_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", 10);
  game_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
  ground_robot_position_pub_ = this->create_publisher<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", 10);
  rfid_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  robot_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  spin_flag_pub_ = 
    this->create_publisher<spin_flag_interfaces::msg::SpinFlag>("spin", 10);

  // create ROS Subscription
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_chassis", 10,
    std::bind(&RMSerialDriver::CmdVelCallback, this, std::placeholders::_1));
  spin_flag_sub_ = this->create_subscription<spin_flag_interfaces::msg::SpinFlag>(
    "spin_flag", 10,
    std::bind(&RMSerialDriver::spinflagCallback, this, std::placeholders::_1));
  last_spin_flag_time_ = this->now();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&RMSerialDriver::timerCallback, this));
  /*############################################################*/

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  aiming_point_.header.frame_id = "move_link";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  // aim_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
  //   "/tracker/target", rclcpp::SensorDataQoS(),
  //   std::bind(&RMSerialDriver::sendArmorData, this, std::placeholders::_1));
  aim_sub_.subscribe(this, "/trajectory/target", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  aim_time_info_sub_.subscribe(this, "/time_info/aim");
  rune_sub_.subscribe(this, "/tracker/rune");
  buff_time_info_sub_.subscribe(this, "/time_info/buff");

  aim_sync_ = std::make_unique<AimSync>(aim_syncpolicy(500), aim_sub_, aim_time_info_sub_);
  aim_sync_->registerCallback(
    std::bind(&RMSerialDriver::sendArmorData, this, std::placeholders::_1, std::placeholders::_2));
  buff_sync_ = std::make_unique<BuffSync>(buff_syncpolicy(1500), rune_sub_, buff_time_info_sub_);
  buff_sync_->registerCallback(
    std::bind(&RMSerialDriver::sendBuffData, this, std::placeholders::_1, std::placeholders::_2));
  // start_time = this->get_clock()->now();
  // start_time_ = start_time.seconds() + 30000;
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

/********************************************************/
/* Receive data                                            */
/********************************************************/
void RMSerialDriver::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData!");
  std::vector<uint8_t> header(1);

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);
      uint8_t sof = header[0];
      // RCLCPP_INFO(get_logger(), "Header: %02X", sof);

      if (sof == 0x5A) {
        std::vector<uint8_t> data;
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), sof);
        ReceivePacket packet = fromVector<ReceivePacket>(data);

        // print calculated crc and received crc
        // uint16_t calculated_crc = crc16::Get_CRC16_Check_Sum(data.data(), static_cast<uint32_t>(data.size() - 2), crc16::CRC16_INIT);
        // uint16_t received_crc = packet.checksum;
        // RCLCPP_ERROR(get_logger(), "Calculated CRC: 0x%04X, Received CRC: 0x%04X", calculated_crc, received_crc);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            // setParam(rclcpp::Parameter("detect_color", 1));
            previous_receive_color_ = packet.detect_color;
          }

          if (packet.reset_tracker) {
            resetTracker();
          }

          if (packet.change_target) {
            changeTarget();
          }

          std_msgs::msg::String task;
          std::string theory_task;
          if (
            (packet.game_time >= 329 && packet.game_time <= 359) ||
            (packet.game_time >= 239 && packet.game_time <= 269)) {
            theory_task = "small_buff";
          } else if (
            (packet.game_time >= 149 && packet.game_time <= 179) ||
            (packet.game_time >= 74 && packet.game_time <= 104) ||
            (packet.game_time > 0 && packet.game_time <= 29)) {
            theory_task = "large_buff";
          } else {
            theory_task = "aim";
          }

          if (packet.task_mode == 0) {
            task.data = theory_task;
          } else if (packet.task_mode == 1) {
            task.data = "aim";
          } else if (packet.task_mode == 2) {
            if (theory_task == "aim") {
              task.data = "auto";
            } else {
              task.data = theory_task;
            }
          } else {
            task.data = "aim";
          }
          task_pub_->publish(task);

          // RCLCPP_DEBUG(
          //   get_logger(), "Game time: %d, Task mode: %d, Theory task: %s", packet.game_time,
          //   packet.task_mode, theory_task.c_str());

          std_msgs::msg::String record_controller;
          record_controller.data = packet.is_play ? "start" : "stop";
          record_controller_pub_->publish(record_controller);

          geometry_msgs::msg::TransformStamped t;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          t.header.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
          t.header.frame_id = "move_link";
          t.child_frame_id = "gimbal_link";
          tf2::Quaternion q;
          packet.pitch = 0.639714775376 * packet.pitch;
          current_pitch = packet.pitch;
          // std::cout << "current_pitch:" << current_pitch << std::endl;
          // packet.yaw = current_yaw;
          q.setRPY(0.0, packet.pitch, packet.yaw);
          t.transform.rotation = tf2::toMsg(q);
          tf_broadcaster_->sendTransform(t);

          geometry_msgs::msg::TransformStamped t1;
          t1.header.stamp = this->now();
          t1.header.frame_id = "base_link";
          t1.child_frame_id = "move_link";
          tf2::Quaternion q1;
          q1.setRPY(0.0, 0.0, -current_yaw);
          t1.transform.rotation = tf2::toMsg(q1);
          t1.transform.translation.x = 0.0;
          t1.transform.translation.y = 0.0;
          t1.transform.translation.z = 0.46;
          tf_broadcaster_->sendTransform(t1);
          
          // transformPointToGunLink();
          // geometry_msgs::msg::TransformStamped t1;
          // t1.header.stamp = point_stamped_in_move_link.header.stamp;  // 使用请求时间而非当前时间
          // t1.header.frame_id = "gimbal_link";
          // t1.child_frame_id = "gun_link";
          // tf2::Quaternion q1;
          // q1.setRPY(0.0, -packet.pitch, 0.0);
          // t1.transform.rotation = tf2::toMsg(q1);
          // t1.transform.translation.x = 0.1104 * cos(packet.pitch);
          // t1.transform.translation.y = 0.0;
          // t1.transform.translation.z = 0.1104 * sin(-packet.pitch);
          // tf_broadcaster_->sendTransform(t1);
          // publish time
          auto_aim_interfaces::msg::TimeInfo aim_time_info;
          buff_interfaces::msg::TimeInfo buff_time_info;
          aim_time_info.header = t.header;
          aim_time_info.time = packet.timestamp;
          buff_time_info.header = t.header;
          buff_time_info.time = packet.timestamp;
          aim_time_info_pub_->publish(aim_time_info);
          buff_time_info_pub_->publish(buff_time_info);

          if (abs(packet.aim_x) > 0.01) {
            aiming_point_.header.stamp = this->now();
            aiming_point_.pose.position.x = packet.aim_x;
            aiming_point_.pose.position.y = packet.aim_y;
            aiming_point_.pose.position.z = packet.aim_z;
            marker_pub_->publish(aiming_point_);
          }
        } 
      }
      else if (sof == 0xA6) {
        std::vector<uint8_t> data_robot_motion;
        data_robot_motion.resize(sizeof(ReceiveRobotMotionData) - 1);
        serial_driver_->port()->receive(data_robot_motion);
        data_robot_motion.insert(data_robot_motion.begin(), sof);
        ReceiveRobotMotionData ros_packet = fromVector<ReceiveRobotMotionData>(data_robot_motion);

        // print calculated crc and received crc
        // uint16_t calculated_crc = crc16::Get_CRC16_Check_Sum(data_robot_motion.data(), static_cast<uint32_t>(data_robot_motion.size() - 2), crc16::CRC16_INIT);
        // uint16_t received_crc = ros_packet.crc;
        // RCLCPP_INFO(get_logger(), "Calculated CRC: 0x%04X, Received CRC: 0x%04X", calculated_crc, received_crc);
        bool crc_ok =
        crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&ros_packet), sizeof(ros_packet));
        if (crc_ok) {
          publishRobotMotion(ros_packet);
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
          // print received data
          // std::ostringstream oss;
          // for (auto byte : data_robot_motion) {
          //   oss << "0x" << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " ";
          // }
          // RCLCPP_INFO(get_logger(), "ros data received: %s", oss.str().c_str());
        }
    }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::publishRobotMotion(ReceiveRobotMotionData & robot_motion)
{
  geometry_msgs::msg::Twist msg;

  spin_flag_interfaces::msg::SpinFlag msg_spin_flag;
  msg_spin_flag.spin_flag = robot_motion.spin_flag;

  // 比赛信息数据
  pb_rm_interfaces::msg::GameStatus msg_game_status;

  msg_game_status.game_progress = robot_motion.game_progress;
  msg_game_status.stage_remain_time = robot_motion.stage_remain_time;

  // 事件数据
  pb_rm_interfaces::msg::EventData msg_eventdata;

  // msg_eventdata.supply_station_front = robot_motion.supply_station_front;
  // msg_eventdata.supply_station_internal = robot_motion.supply_station_internal;
  // msg_eventdata.supply_zone = robot_motion.supply_zone;
  // msg_eventdata.center_gain_zone = robot_motion.center_gain_zone;

  // msg_eventdata.small_energy = robot_motion.small_energy;
  // msg_eventdata.big_energy = robot_motion.big_energy;

  // msg_eventdata.circular_highland = robot_motion.circular_highland;
  // msg_eventdata.trapezoidal_highland_3 = robot_motion.trapezoidal_highland_3;
  // msg_eventdata.trapezoidal_highland_4 = robot_motion.trapezoidal_highland_4;

  // msg_eventdata.base_virtual_shield_remaining = robot_motion.base_virtual_shield_remaining;

  // 全场机器人hp信息数据
  pb_rm_interfaces::msg::GameRobotHP msg_robot_hp;

  // msg_robot_hp.red_1_robot_hp = robot_motion.red_1_robot_hp;
  // msg_robot_hp.red_2_robot_hp = robot_motion.red_2_robot_hp;
  // msg_robot_hp.red_3_robot_hp = robot_motion.red_3_robot_hp;
  // msg_robot_hp.red_4_robot_hp = robot_motion.red_4_robot_hp;
  // msg_robot_hp.red_5_robot_hp = robot_motion.red_5_robot_hp;
  // msg_robot_hp.red_7_robot_hp = robot_motion.red_7_robot_hp;
  // msg_robot_hp.red_outpost_hp = robot_motion.red_outpost_hp;
  // msg_robot_hp.red_base_hp = robot_motion.red_base_hp;

  // msg_robot_hp.blue_1_robot_hp = robot_motion.blue_1_robot_hp;
  // msg_robot_hp.blue_2_robot_hp = robot_motion.blue_2_robot_hp;
  // msg_robot_hp.blue_3_robot_hp = robot_motion.blue_3_robot_hp;
  // msg_robot_hp.blue_4_robot_hp = robot_motion.blue_4_robot_hp;
  // msg_robot_hp.blue_5_robot_hp = robot_motion.blue_5_robot_hp;
  // msg_robot_hp.blue_7_robot_hp = robot_motion.blue_7_robot_hp;
  // msg_robot_hp.blue_outpost_hp = robot_motion.blue_outpost_hp;
  // msg_robot_hp.blue_base_hp = robot_motion.blue_base_hp;

  // 地面机器人位置数据
  pb_rm_interfaces::msg::GroundRobotPosition msg_ground_robot_position;

  // msg_ground_robot_position.hero_x = robot_motion.hero_x;
  // msg_ground_robot_position.hero_y = robot_motion.hero_y;

  // msg_ground_robot_position.engineer_x = robot_motion.engineer_x;
  // msg_ground_robot_position.engineer_y = robot_motion.engineer_y;

  // msg_ground_robot_position.standard_3_x = robot_motion.standard_3_x;
  // msg_ground_robot_position.standard_3_y = robot_motion.standard_3_y;

  // msg_ground_robot_position.standard_4_x = robot_motion.standard_4_x;
  // msg_ground_robot_position.standard_4_y = robot_motion.standard_4_y;

  // msg_ground_robot_position.standard_5_x = robot_motion.standard_5_x;
  // msg_ground_robot_position.standard_5_y = robot_motion.standard_5_y;

  // RFID 状态数据
  pb_rm_interfaces::msg::RfidStatus msg_rfid_status;

  // msg_rfid_status.base_gain_point = robot_motion.base_gain_point;
  // msg_rfid_status.central_highland_gain_point = robot_motion.central_highland_gain_point;
  // msg_rfid_status.enemy_central_highland_gain_point = robot_motion.enemy_central_highland_gain_point;
  // msg_rfid_status.friendly_trapezoidal_highland_gain_point =robot_motion.friendly_trapezoidal_highland_gain_point;
  // msg_rfid_status.enemy_trapezoidal_highland_gain_point = robot_motion.enemy_trapezoidal_highland_gain_point;
  // msg_rfid_status.friendly_fly_ramp_front_gain_point = robot_motion.friendly_fly_ramp_front_gain_point;
  // msg_rfid_status.friendly_fly_ramp_back_gain_point = robot_motion.friendly_fly_ramp_back_gain_point;
  // msg_rfid_status.enemy_fly_ramp_front_gain_point = robot_motion.enemy_fly_ramp_front_gain_point;
  // msg_rfid_status.enemy_fly_ramp_back_gain_point = robot_motion.enemy_fly_ramp_back_gain_point;
  // msg_rfid_status.friendly_central_highland_lower_gain_point = robot_motion.friendly_central_highland_lower_gain_point;
  // msg_rfid_status.friendly_central_highland_upper_gain_point = robot_motion.friendly_central_highland_upper_gain_point;
  // msg_rfid_status.enemy_central_highland_lower_gain_point = robot_motion.enemy_central_highland_lower_gain_point;
  // msg_rfid_status.enemy_central_highland_upper_gain_point = robot_motion.enemy_central_highland_upper_gain_point;
  // msg_rfid_status.friendly_highway_lower_gain_point = robot_motion.friendly_highway_lower_gain_point;
  // msg_rfid_status.friendly_highway_upper_gain_point = robot_motion.friendly_highway_upper_gain_point;
  // msg_rfid_status.enemy_highway_lower_gain_point = robot_motion.enemy_highway_lower_gain_point;
  // msg_rfid_status.enemy_highway_upper_gain_point = robot_motion.enemy_highway_upper_gain_point;
  msg_rfid_status.friendly_fortress_gain_point = robot_motion.friendly_fortress_gain_point;
  // msg_rfid_status.friendly_outpost_gain_point = robot_motion.friendly_outpost_gain_point;
  msg_rfid_status.friendly_supply_zone_non_exchange = robot_motion.friendly_supply_zone_non_exchange;
  msg_rfid_status.friendly_supply_zone_exchange = robot_motion.friendly_supply_zone_exchange;
  // msg_rfid_status.friendly_big_resource_island = robot_motion.friendly_big_resource_island;
  // msg_rfid_status.enemy_big_resource_island = robot_motion.enemy_big_resource_island;
  msg_rfid_status.center_gain_point = robot_motion.center_gain_point;

  // 机器人状态数据
  pb_rm_interfaces::msg::RobotStatus msg_robot_status;

  // msg_robot_status.robot_id = robot_motion.robot_id;
  // msg_robot_status.robot_level = robot_motion.robot_level;
  msg_robot_status.current_hp = robot_motion.current_hp;
  // msg_robot_status.ARMOR_HIT = robot_motion.ARMOR_HIT;
  // msg_robot_status.maximum_hp = robot_motion.maximum_hp;
  // msg_robot_status.is_hp_deduced = robot_motion.is_hp_deduced;
  msg_robot_status.projectile_allowance_17mm = robot_motion.projectile_allowance_17mm;
  // msg_robot_status.shooter_barrel_cooling_value = robot_motion.shooter_barrel_cooling_value;
  // msg_robot_status.shooter_barrel_heat_limit = robot_motion.shooter_barrel_heat_limit;
  // msg_robot_status.shooter_barrel_heat_limit = robot_motion.shooter_barrel_heat_limit;
  // msg_robot_status.shooter_barrel_heat_limit = robot_motion.shooter_barrel_heat_limit;
  msg_robot_status.shooter_17mm_1_barrel_heat = robot_motion.shooter_17mm_1_barrel_heat;
  // msg_robot_status.robot_pos_x = robot_motion.robot_pos_x;
  // msg_robot_status.robot_pos_y = robot_motion.robot_pos_y;
  // msg_robot_status.robot_pos_angle = robot_motion.robot_pos_angle;
  msg_robot_status.armor_id = robot_motion.armor_id;
  msg_robot_status.hp_deduction_reason = robot_motion.hp_deduction_reason;
  // msg_robot_status.projectile_allowance_17mm_1 = robot_motion.projectile_allowance_17mm_1;
  // msg_robot_status.remaining_gold_coin = robot_motion.remaining_gold_coin;
  // msg_robot_status.spin_flag = robot_motion.spin_flag;

  // 云台状态数据
  // sensor_msgs::msg::JointState msg_joint_state;

  // msg_joint_state.position.resize(2);
  // msg_joint_state.name.resize(2);
  // msg_joint_state.header.stamp = now();

  // msg_joint_state.name[0] = "gimbal_pitch_joint";
  // msg_joint_state.position[0] = msg_joint_state.pitch;

  // msg_joint_state.name[1] = "gimbal_yaw_joint";
  // msg_joint_state.position[1] = msg_joint_state.yaw;

  robot_motion_pub_->publish(msg);
  spin_flag_pub_->publish(msg_spin_flag);
  game_status_pub_->publish(msg_game_status);
  event_data_pub_->publish(msg_eventdata);
  all_robot_hp_pub_->publish(msg_robot_hp);
  ground_robot_position_pub_->publish(msg_ground_robot_position);
  rfid_status_pub_->publish(msg_rfid_status);
  robot_status_pub_->publish(msg_robot_status);
  // joint_state_pub_->publish(msg_joint_state); 
}

/********************************************************/
/* Send data                                            */
/********************************************************/
void RMSerialDriver::sendArmorData(
  const auto_aim_interfaces::msg::Trajectory::ConstSharedPtr msg,
  const auto_aim_interfaces::msg::TimeInfo::ConstSharedPtr time_info)
{
  // RCLCPP_INFO(get_logger(), "Start send ros data!");
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};
  // std::cout << "++++++++++++++++++++" << std::endl;
  try {
    SendPacket packet;
    // start_time = this->now();
    // if (start_time.seconds() - start_time_ > 0) {
    // if (msg->need_navigation) {
    //   packet.state = 0;
    //   packet.track = 1;
    //   packet.change_pitch = 0;
    //   packet.change_yaw = 0;
    //   packet.speed_x = speed_x;
    //   packet.speed_y = speed_y;
    //   packet.angular = angular;
    //   // packet.spin_flag = 0;
    //   packet.spin_flag = spin_flag;
    // } else {
    //   packet.state = msg->fire;
    //   packet.track = msg->tracking;
    //   packet.change_pitch = msg->pitch_w;
    //   packet.change_yaw = msg->yaw_w;
    //   current_yaw = msg->current_yaw;
    //   packet.speed_x = 0;
    //   packet.speed_y = 0;
    //   packet.angular = angular;
    //   // packet.spin_flag = 0;
    //   packet.spin_flag = spin_flag;
    // }
    //trajectory_msg.need_navigation = 0;
    packet.state = msg->fire;
    packet.track = msg->tracking;
    packet.change_pitch = msg->pitch_w;
    packet.change_yaw = msg->yaw_w;
    // current_yaw = msg->current_yaw;
    // packet.speed_x = 0;
    // packet.speed_y = 0;
    // packet.angular = 0;
    // std::cout << "111" << std::endl;
    // } else {
    // packet.track = 1;
    // if (current_pitch > 0.13) {
    //   up_or_down = 0.05;
    //   std::cout << "1111" << std::endl;
    // } else if (current_pitch < -0.13) {
    //   up_or_down = 0.05;
    //   std::cout << "2222" << std::endl;
    // }
    // if (msg->tracking == 0) {
    //   packet.change_yaw = -0.05;
    //   packet.change_pitch = up_or_down;
    //   std::cout << "3333" << std::endl;
    // } else if (msg->tracking == 2) {
    //   packet.change_yaw = 0.05;
    //   packet.change_pitch = up_or_down;
    //   std::cout << "4444" << std::endl;
    // } else {
    //   packet.change_yaw = msg->yaw_w;
    //   packet.change_pitch = msg->pitch_w;
    //   std::cout << "5555" << std::endl;
    // }
    // std::cout << msg->tracking << " change_yaw:" << packet.change_yaw << " change_pitch:" << packet.change_pitch << " current_pitch:" << current_pitch << " up_or_down:" << up_or_down << std::endl;
    // packet.state = 0;
    // packet.track = 1;
    // packet.change_pitch = 0;
    // packet.change_yaw = 0;
    packet.speed_x = speed_x;
    packet.speed_y = speed_y;
    packet.angular = angular;
    packet.spin_flag = spin_flag;
    // }
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);
    // std::cout << "**********" << std::endl;
    serial_driver_->port()->send(data);
    // std::cout << "-------------" << std::endl;

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
  // std::this_thread::sleep_for(std::chrono::milliseconds(5)); //使当前线程暂停 5 毫秒

}

void RMSerialDriver::sendBuffData(
  const buff_interfaces::msg::Rune::ConstSharedPtr rune,
  const buff_interfaces::msg::TimeInfo::ConstSharedPtr time_info)
{
  try {
    SendPacket packet;
    packet.state = rune->tracking ? 2 : 0;
    // packet.id = rune->offset_id;
    // packet.armors_num = rune->offset_id;
    // packet.x = rune->position.x;
    // packet.y = rune->position.y;
    // packet.z = rune->position.z;
    // packet.yaw = rune->theta;
    // packet.vx = rune->a;
    // packet.vy = rune->b;
    // packet.vz = rune->w;
    // packet.v_yaw = 0.0;
    // packet.r1 = 0.0;
    // packet.r2 = 0.0;
    // packet.dz = 0.0;
    // packet.cap_timestamp = time_info->time;
    // if (rune->w == 0) {
    //   packet.t_offset = 0;
    // } else {
    //   int T = abs(2 * sove_PI / rune->w * 1000);
    //   int offset = (rune->t_offset - time_info->time % T) % T;
    //   if (offset < 0) {
    //     packet.t_offset = T + offset;
    //   }
    // }
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - rune->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
    // std::this_thread::sleep_for(std::chrono::milliseconds(50)); //使当前线程暂停 5 毫秒

}

void RMSerialDriver::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  speed_x = msg->linear.x;
  speed_y = msg->linear.y;
  angular = msg->angular.z;
  RCLCPP_INFO(get_logger(), "Received velocity command: speed_x = %.2f, speed_y = %.2f, angular: %.2f",
              msg->linear.x, msg->linear.y, msg->angular.z);
}
void RMSerialDriver::spinflagCallback(const spin_flag_interfaces::msg::SpinFlag::SharedPtr msg)
{
  spin_flag = msg->spin_flag;
  last_spin_flag_time_ = this->now();
  // RCLCPP_INFO(get_logger(), "Received spin flag: %d", msg->spin_flag);
}
void RMSerialDriver::timerCallback() {
    auto now = this->now();
    auto time_since_last_msg = now.seconds() - last_spin_flag_time_.seconds();
    
    if (time_since_last_msg > 5.0) {
        spin_flag = 0;
    // RCLCPP_WARN(get_logger(), "No spin_flag message for 5s, set to false");
    }
}
void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or "
        "hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

void RMSerialDriver::changeTarget()
{
  if (!change_target_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping target change");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  change_target_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Change target!");
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
// source install/setup.bash
// ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
