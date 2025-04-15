// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/trajectory.hpp"
#include "auto_aim_interfaces/msg/time_info.hpp"
#include "buff_interfaces/msg/rune.hpp"
#include "buff_interfaces/msg/time_info.hpp"

// ROS message
#include <example_interfaces/msg/float64.hpp>
#include <example_interfaces/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cstring>
#include <pb_rm_interfaces/msg/event_data.hpp>
#include <pb_rm_interfaces/msg/game_robot_hp.hpp>
#include <pb_rm_interfaces/msg/game_status.hpp>
#include <pb_rm_interfaces/msg/ground_robot_position.hpp>
#include <pb_rm_interfaces/msg/rfid_status.hpp>
#include <pb_rm_interfaces/msg/robot_state_info.hpp>
#include <pb_rm_interfaces/msg/robot_status.hpp>
#include <spin_flag_interfaces/msg/spin_flag.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rm_serial_driver/packet.hpp"

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void receiveData();

  // void sendArmorData(const auto_aim_interfaces::msg::Target::ConstSharedPtr msg);

  void sendArmorData(
    const auto_aim_interfaces::msg::Trajectory::ConstSharedPtr msg,
    const auto_aim_interfaces::msg::TimeInfo::ConstSharedPtr time_info);

  void sendBuffData(
    const buff_interfaces::msg::Rune::ConstSharedPtr msg,
    const buff_interfaces::msg::TimeInfo::ConstSharedPtr time_info);

/*############################################################*/
  // Send ROS data
  void sendData(); 
  SendPacket packet;

  float speed_x = 0, speed_y = 0, angular = 0;
  uint8_t spin_flag = 0;

  void publishRobotMotion(ReceiveRobotMotionData & ReceiveRobotMotionData);

  // 回调函数，订阅cmd_vel_chassis和spin_flag
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void spinflagCallback(const spin_flag_interfaces::msg::SpinFlag::SharedPtr msg);
  void timerCallback();

/*############################################################*/
  void reopenPort();

  void setParam(const rclcpp::Parameter & param);

  void resetTracker();

  void changeTarget();

  double sove_PI = 3.141592653589793238, current_yaw, current_pitch, start_time_;
  float up_or_down;
  rclcpp::Time start_time;
  // std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // Param client to set detect_colr
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  // Service client to reset tracker
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  // Service client to change target
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr change_target_client_;

  // Aimimg point receiving from serial port for visualization
  visualization_msgs::msg::Marker aiming_point_;

  // Broadcast tf from move_link to gimbal_link
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr aim_sub_;

  message_filters::Subscriber<auto_aim_interfaces::msg::Trajectory> aim_sub_;
  message_filters::Subscriber<auto_aim_interfaces::msg::TimeInfo> aim_time_info_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    auto_aim_interfaces::msg::Trajectory, auto_aim_interfaces::msg::TimeInfo>
    aim_syncpolicy;
  typedef message_filters::Synchronizer<aim_syncpolicy> AimSync;
  std::shared_ptr<AimSync> aim_sync_;

  message_filters::Subscriber<buff_interfaces::msg::Rune> rune_sub_;
  message_filters::Subscriber<buff_interfaces::msg::TimeInfo> buff_time_info_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    buff_interfaces::msg::Rune, buff_interfaces::msg::TimeInfo>
    buff_syncpolicy;
  typedef message_filters::Synchronizer<buff_syncpolicy> BuffSync;
  std::shared_ptr<BuffSync> buff_sync_;

  // For debug usage
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::thread receive_thread_;

  // Task message
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;

  // Time message
  rclcpp::Publisher<auto_aim_interfaces::msg::TimeInfo>::SharedPtr aim_time_info_pub_;
  rclcpp::Publisher<buff_interfaces::msg::TimeInfo>::SharedPtr buff_time_info_pub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr record_controller_pub_;

  // ROS Publish
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStateInfo>::SharedPtr robot_state_info_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::EventData>::SharedPtr event_data_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr all_robot_hp_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_motion_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GroundRobotPosition>::SharedPtr
    ground_robot_position_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<spin_flag_interfaces::msg::SpinFlag>::SharedPtr spin_flag_pub_;

  //ROS Subscribe
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<spin_flag_interfaces::msg::SpinFlag>::SharedPtr spin_flag_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_spin_flag_time_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
