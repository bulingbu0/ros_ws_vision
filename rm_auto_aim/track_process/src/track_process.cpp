// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Matrix3x3.h"

// #include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
// #include <rclcpp/utilities.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/core.hpp>
// #include <opencv2/core/types.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>


// C++ system
#include <cstdint>
// #include <functional>
// #include <map>
// #include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>


#include "track_process.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "auto_aim_interfaces/msg/target.hpp"
// #include "auto_aim_interfaces/msg/trajectory.hpp"
// #include "auto_aim_interfaces/msg/is_reach_goal.hpp"


namespace rm_auto_aim
{
TrajectoryNode::TrajectoryNode(const rclcpp::NodeOptions & options)
: Node("track_process", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrajectoryNode!");

  last_time = this->get_clock()->now();
  
  // 对v_yaw，dz取平均值
  v_yaw_vec = {0.0, 0.0, 0.0, 0.0, 0.0};
  dz_vec = {0.0, 0.0, 0.0, 0.0, 0.0};
  // 弹道解算的参数
  m = 3.2e-3;
  d = 18.6e-3;
  Cd = 0.47;
  rho = 1.169;
  g = 9.778;
  // 弹速
  v = 24.0;
  // 目标点的偏置
  dis_x = 0.0;
  dis_z = 0.0;
  sove_PI = 3.141592653589793238;
  r = d / 2;
  A = sove_PI * r * r;
  k = 0.5 * Cd * rho * A;

  last_u = 750;
  last_receive_reach_goal_time = 0;
  t_flight = 0;

  detect_yaw_ = 0;

  current_yaw = 0;
  // 装甲板跳变的时刻
  scopperil_seconds = 0;
  // 开火
  fire = 0;
  // 停止开火
  stop_scopperil_fire = 0;
  // 装甲板跳变次数
  scopperil_sount = 0;

  // 前一时刻的xyz
  last_x = 0;
  last_y = 0;
  last_z = -0.0;
  // dz初值
  solve_dz = 10;
  // 为检测到目标则发布0
  should_zero = true;
  // 目标是否为小陀螺
  is_scopperil = false;

  camera_matrix = (cv::Mat_<double>(3, 3) <<
    1775.67764,    0.     ,  730.347017,
      0.     , 1775.35353,  565.335862,
      0.     ,    0.     ,    1.     );

  rvec = cv::Mat::zeros(3, 1, CV_64F);
  tvec = cv::Mat::zeros(3, 1, CV_64F);

  distortion_coeffs = (cv::Mat_<double>(1, 5) <<
      -0.107062425, 0.149252511, -0.0000797442932, 0.000370530134, -0.0471226324);
  gimbal_rotate_ = this->declare_parameter("gimbal_rotate", false);

  target_frame_ = this->declare_parameter("target_frame", "move_link");
  // 预测时间
  scopperil_predict_dt_ = scopperil_predict_dt = this->declare_parameter("scopperil_predict_dt", 0.33);
  move_predict_dt_ = move_predict_dt = this->declare_parameter("move_predict_dt", 0.33);
  // 弹道解算发布者
  trajectory_pub_ = this->create_publisher<auto_aim_interfaces::msg::Trajectory>(
    "/trajectory/target", rclcpp::SensorDataQoS());
  
  // 将move_link的点转化到gun_link上的tf变换
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), std::chrono::seconds(30));
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // tracker/target订阅者
  track_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
     std::bind(&TrajectoryNode::trackCallback, this, std::placeholders::_1));
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////快速注释线
//     //  debug发布者
//   draw_img_pub_ = image_transport::create_publisher(this, "/detector/draw_img");
//   img_thread_ = std::thread(&TrajectoryNode::_img_sub_, this);
//   std::string videoPath = "/home/humble/ros_ws_vision1/output.avi";
//   // out.open(videoPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(1440, 1080));
// // ////////////////////////////快速注释线
//   // odemetry_thread_ = std::thread(&TrajectoryNode::_odemetry_sub_, this);
//   // gimbal_rotate_thread_ = std::thread(&TrajectoryNode::_gimbal_rotate_sub_, this);
// //   ////////////////////////////快速注释线
// }

// TrajectoryNode::~TrajectoryNode() {
// // ////////////////////////////快速注释线
//   // if (odemetry_thread_.joinable()) {
//   //   odemetry_thread_.join();
//   // }
//   // if (gimbal_rotate_thread_.joinable()) {
//   //   gimbal_rotate_thread_.join();
//   // }
// //   ////////////////////////////快速注释线
//   if (out.isOpened()) {
//     out.release();
//   }
//   if (img_thread_.joinable()) {
//     img_thread_.join();
//   }
// }

// // ////////////////////////////快速注释线
// // void TrajectoryNode::_odemetry_sub_(){
// //   odemetry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
// //     "/Odometry", rclcpp::SensorDataQoS(),
// //     std::bind(&TrajectoryNode::odom_callback, this, std::placeholders::_1));
// // }
// // // 获取雷达的yaw
// // void TrajectoryNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
// //   tf2::Quaternion q(msg->pose.pose.orientation.x,
// //                     msg->pose.pose.orientation.y,
// //                     msg->pose.pose.orientation.z,
// //                     msg->pose.pose.orientation.w);
// //   tf2::Matrix3x3 m(q);
// //   m.getRPY(current_roll, current_pitch, current_yaw);
// // }

// // void TrajectoryNode::_gimbal_rotate_sub_(){
// //   gimbal_rotate_sub_ = this->create_subscription<auto_aim_interfaces::msg::IsReachGoal>(
// //     "/is_reach_goal", 10,
// //     std::bind(&TrajectoryNode::GimbalRotateCallback, this, std::placeholders::_1));
// // }

// // void TrajectoryNode::GimbalRotateCallback(const auto_aim_interfaces::msg::IsReachGoal::SharedPtr msg) {
// //   gimbal_rotate_ = msg->is_reach_goal;
// //   last_receive_reach_goal_time = last_time.seconds();
// // }
// //  ////////////////////////////快速注释线

// void TrajectoryNode::_img_sub_(){
//   img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//     "/detector/result_img", rclcpp::SensorDataQoS(),
//     std::bind(&TrajectoryNode::imageCallback, this, std::placeholders::_1));
// }

// void TrajectoryNode::imageCallback(
//   const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {

//   auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

//   geometry_msgs::msg::Point point_in_gun_link, point_in_camera_optical_frame;
  
//   point_in_gun_link.x = solve_xc;
//   point_in_gun_link.y = solve_yc;
//   point_in_gun_link.z = solve_zc;

//   point_in_camera_optical_frame = transformPoint(point_in_gun_link, "camera_optical_frame", "gun_link");

//   // 三维点 (X, Y, Z)，假设矩形的中心点
//   cv::Point3f point_3d1(point_in_camera_optical_frame.x, point_in_camera_optical_frame.y, point_in_camera_optical_frame.z);  // 使用构造函数
  
//   // 将三维点投影到像素坐标系
//   std::vector<cv::Point3f> object_points1 = {point_3d1};
//   std::vector<cv::Point2f> image_points1;

//   // 使用 OpenCV 的函数进行投影
//   cv::projectPoints(object_points1, rvec, tvec, camera_matrix, distortion_coeffs, image_points1);


//   point_in_gun_link.x = solve_xa1;
//   point_in_gun_link.y = solve_ya1;
//   point_in_gun_link.z = solve_zc;

//   point_in_camera_optical_frame = transformPoint(point_in_gun_link, "camera_optical_frame", "gun_link");

//   // 三维点 (X, Y, Z)，假设矩形的中心点
//   cv::Point3f point_3d2(point_in_camera_optical_frame.x, point_in_camera_optical_frame.y, point_in_camera_optical_frame.z);  // 使用构造函数
  
//   // 将三维点投影到像素坐标系
//   std::vector<cv::Point3f> object_points2 = {point_3d2};
//   std::vector<cv::Point2f> image_points2;

//   // 使用 OpenCV 的函数进行投影
//   cv::projectPoints(object_points2, rvec, tvec, camera_matrix, distortion_coeffs, image_points2);


//   try{
//       u_ = image_points1[0].x;
//       v_ = image_points1[0].y;
//       cv::Point point1(static_cast<int>(u_), static_cast<int>(v_));
//       cv::circle(img, point1, 10, CV_RGB(255, 0, 255), -1);

//       float u2 = image_points2[0].x;
//       float v2 = image_points2[0].y;
//       cv::Point point2(static_cast<int>(u2), static_cast<int>(v2));
//       cv::circle(img, point2, 10, CV_RGB(255, 0, 0), -1);

//       cv::circle(img, cv::Point(720, 600), 10, CV_RGB(100, 100, 255), 2);
//       std::string A_ = std::to_string(fire);
//       std::string B_ = std::to_string(which_if);
//       std::string C_ = std::to_string(pitch_w);
//       std::string D_ = std::to_string(yaw_w);
//       std::string E_ = std::to_string(detect_yaw_);
//       // 往下动为正 往上动为负
//       cv::putText(
//       img, A_, cv::Point(720, 500), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(150, 200, 180), 2);
//       cv::putText(
//       img, B_, cv::Point(720, 400), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(150, 100, 255), 2);
//       cv::putText(
//       img, C_, cv::Point(820, 500), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 100, 100), 2);
//       cv::putText(
//       img, D_, cv::Point(820, 550), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 100, 100), 2);
//       cv::putText(
//       img, E_, cv::Point(420, 550), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 100, 100), 2);
//   } catch (const cv_bridge::Exception& e){
//       return;
//   }
//   cv::cvtColor(img, rgbFrame, cv::COLOR_BGR2RGB); // 转换为 RGB 格式
//   out.write(rgbFrame);
//   draw_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
//   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////快速注释线
}

 
geometry_msgs::msg::Point TrajectoryNode::transformPoint(geometry_msgs::msg::Point point_in_original_link, 
          std::string point_in_target, std::string point_in_original)
{
  geometry_msgs::msg::PointStamped point_in_target_link_, point_stamped_in_original_link;
  geometry_msgs::msg::Point p;
  // 创建一个PointStamped对象，将move_link坐标系中的点转为时间戳信息
  point_stamped_in_original_link.header.frame_id = point_in_original;  // 设置源坐标系
  rclcpp::Time current_time = this->get_clock()->now();  // 获取当前时间
  point_stamped_in_original_link.header.stamp = current_time;  // 将请求时间戳设置为当前时间
  point_stamped_in_original_link.point = point_in_original_link;

  try {
    geometry_msgs::msg::TransformStamped transform_stamped = tf2_buffer_->lookupTransform(
        point_in_target,  // 目标坐标系
        point_in_original,        // 源坐标系
        tf2::TimePointZero,  // 自动选择最新可用变换
        tf2::durationFromSec(0.1)  // 超时时间
    );
    tf2::doTransform(point_stamped_in_original_link, point_in_target_link_, transform_stamped);

  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Error while transforming: %s", ex.what());
  }
  p = point_in_target_link_.point;
  return p;
}
// 弹道解算
double TrajectoryNode::SolveTrajectory(double theta, double x){
  double cos_theta = cos(theta);
  double term1 = x * tan(theta);
  double term2 = (m * g * x) / (k * v * cos_theta);
  double log_term = 1 - (k * x) / (m * v * cos_theta);
  double term3 = (m * m * g) * std::log(log_term) / (k * k);
  return term1 + term2 + term3;
}

// 预测、选板、弹道解算、发布
void TrajectoryNode::trackCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  // 初始状态或未追踪到目标则发布0
  if (last_x == 0 && last_y == 0 && last_z == 0) {
    should_zero = true;
  } else if (msg->position.x == last_x && msg->position.y == last_y && msg->position.z == last_z) {
    should_zero = true;
  } else {
    should_zero = false;
  }
  last_x = msg->position.x;
  last_y = msg->position.y;
  last_z = solve_zc = msg->position.z;

  detect_yaw_ = msg->detect_yaw;

  last_time = this->now();
  // if (last_time.seconds() - last_receive_reach_goal_time > 1.2) {
  //   gimbal_rotate_ = false;
  // }
  // std::cout << "    gimbal_rotate_: " << gimbal_rotate_ << std::endl;
  // RCLCPP_WARN(rclcpp::get_logger("track_process"), "last_time.seconds() %f ", last_time.seconds());

  should_zero = gimbal_rotate_ ? should_zero : false;
  if (!should_zero) {
    last_u = u_;

    last_yaw = solve_yaw = msg->yaw;
    solve_vx = msg->velocity.x;
    solve_vy = msg->velocity.y;

    solve_v_yaw = msg->v_yaw;
    
    solve_r1 = msg->radius_1;
    solve_r2 = msg->radius_2;
    solve_armors_num = msg->armors_num;
    solve_dz = msg->dz;
    solve_tracking = msg->tracking;

    
    // 对v_yaw dz取平均值
    double sum_v_yaw = 0, sum_dz = 0;
    if (solve_tracking) {
      v_yaw_vec.erase(v_yaw_vec.begin());
      v_yaw_vec.push_back(solve_v_yaw);

      dz_vec.erase(dz_vec.begin());
      dz_vec.push_back(solve_dz);

      for (double value : v_yaw_vec) {
            sum_v_yaw += value;
      }
      for (double value : dz_vec) {
            sum_dz += value;
      }
    } else {
      v_yaw_vec = {0.0, 0.0, 0.0, 0.0, 0.0};
      dz_vec = {0.0, 0.0, 0.0, 0.0, 0.0};
    }
    solve_v_yaw = sum_v_yaw / 5;
    solve_dz = sum_dz / 5;
    turn_right = (solve_v_yaw > 0) ? 1 : -1;

    solve_xc = last_x + move_predict_dt_ * solve_vx;
    solve_yc = last_y + move_predict_dt_ * solve_vy;
    solve_yaw += scopperil_predict_dt_ * solve_v_yaw;

    // 计算车的距离和z
    solve_dis = Eigen::Vector2d(solve_xc, solve_yc).norm();

    // 射击的容错角度
    shoot_yaw = (solve_dis < 5) ? 0.85 / solve_dis : 0.017; // 0.085
    shoot_pitch = (solve_dis < 4) ? 0.065 / solve_dis : 0.01625;


    // 如果连续4次装甲板跳变的时间都小于0.6s，则认为目标开启了小陀螺
    if (msg->is_scopperil) {
      scopperil_seconds = last_time.seconds();
    }
    if (last_time.seconds() - scopperil_seconds < 0.6) {
      scopperil_sount ++;
      if (scopperil_sount > 4) {
        scopperil_sount = 4;
        is_scopperil = true;
      }
    } else {
      scopperil_sount --;
      if (scopperil_sount < 0) {
        scopperil_sount = 0;
        is_scopperil = false;
      }
    }

    // 计算第一块装甲板和车的中心在move_link下的位置
    solve_xa1 = solve_xc - solve_r1 * cos(solve_yaw);
    solve_ya1 = solve_yc - solve_r1 * sin(solve_yaw);

    geometry_msgs::msg::Point point_in_move_link, point_in_gun_link;

    fire = false;
    if (is_scopperil) {
      // 如果开启小陀螺则瞄准两相邻装甲板的中心，否则瞄观测到的装甲板的中心
      // solve_za = solve_zc;
      solve_za = solve_zc + 0.5 * solve_dz - (0.3 / solve_dis);

      // 两相邻装甲板的角度
      one_angle = (turn_right * sove_PI * 2 / solve_armors_num);


      // 计算第另外两块装甲板在move_link下的位置
      solve_xa2 = solve_xc - solve_r2 * cos(solve_yaw - one_angle);
      solve_ya2 = solve_yc - solve_r2 * sin(solve_yaw - one_angle);
      solve_xa3 = solve_xc - solve_r1 * cos(solve_yaw - 2 * one_angle);
      solve_ya3 = solve_yc - solve_r1 * sin(solve_yaw - 2 * one_angle);

      // std::cout << "   solve_xa1: " << solve_xa1 << " solve_ya1: " << solve_ya1 << std::endl;
      // std::cout << "   solve_xa2: " << solve_xa2 << " solve_ya2: " << solve_ya2 << std::endl;
      // std::cout << "   solve_xa3: " << solve_xa3 << " solve_ya3: " << solve_ya3 << std::endl;
      // std::cout << "   solve_za: " << solve_za << std::endl;

      point_in_move_link.x = solve_xc;
      point_in_move_link.y = solve_yc;
      point_in_move_link.z = solve_zc;
      point_in_gun_link = transformPoint(point_in_move_link, "gun_link", "move_link");
      solve_xc = point_in_gun_link.x;
      solve_yc = point_in_gun_link.y;
      solve_zc = point_in_gun_link.z;

      point_in_move_link.x = solve_xa1;
      point_in_move_link.y = solve_ya1;
      point_in_move_link.z = solve_za;
      point_in_gun_link = transformPoint(point_in_move_link, "gun_link", "move_link");
      solve_xa1 = point_in_gun_link.x;
      solve_ya1 = point_in_gun_link.y;

      // 将另外两块装甲板在move_link下的点（因为x、y，z因为是相同的，所以变一次就足够）变换到gun_link下来
      point_in_move_link.x = solve_xa2;
      point_in_move_link.y = solve_ya2;
      // point_in_move_link.z = solve_za;
      point_in_gun_link = transformPoint(point_in_move_link, "gun_link", "move_link");
      solve_xa2 = point_in_gun_link.x;
      solve_ya2 = point_in_gun_link.y;

      point_in_move_link.x = solve_xa3;
      point_in_move_link.y = solve_ya3;
      // point_in_move_link.z = solve_za;
      point_in_gun_link = transformPoint(point_in_move_link, "gun_link", "move_link");
      solve_xa3 = point_in_gun_link.x;
      solve_ya3 = point_in_gun_link.y;
      solve_za = point_in_gun_link.z;

      solve_yaw1 = atan2(solve_ya1, solve_xa1);
      solve_yaw2 = atan2(solve_ya2, solve_xa2);
      solve_yaw3 = atan2(solve_ya3, solve_xa3);

      pitch_w = atan2(solve_za, solve_xa1);

      // 如果小陀螺的速度很低并且距离很近，则云台会跟随装甲板移动而移动
      if (abs(solve_v_yaw) < 1.6 && (solve_dis <= 2.8)) {
        if (turn_right * solve_yaw < 0.6) {
          solve_xc = solve_xa1;
          solve_yc = solve_ya1;
          which_if = 11;
        } else {
          solve_xc = solve_xa2;
          solve_yc = solve_ya2;
          which_if = 22;
        }
        fire = abs(atan2(solve_yc, solve_xc)) < shoot_yaw ? 1 : 0;
      } else if (abs(solve_v_yaw) >= 13.0 && (abs(pitch_w) < shoot_pitch)) {
        // 如果小陀螺速度非差快，则持续开火
        which_if = 33;
        fire = true;
      } else if ((abs(solve_v_yaw) >= 1.6 || solve_dis > 2.8)) {
        if ((abs(solve_yaw1) < shoot_yaw || abs(solve_yaw2) < shoot_yaw || abs(solve_yaw3) < shoot_yaw) &&
          abs(pitch_w) < shoot_pitch) {
          // 如果小陀螺速度适中 则预测的三块装甲板转到在相机中心时开火 并更改相应的debug
          stop_scopperil_fire ++;
          if (stop_scopperil_fire < 12) {fire = true;}
          if (abs(solve_yaw1) < shoot_yaw) {which_if = 40;}
          else if (abs(solve_yaw2) < shoot_yaw) {which_if = 41;}
          else if (abs(solve_yaw3) < shoot_yaw) {which_if = 42;}
        } else {
          // 打击转速并不快的小陀螺时因预测错误而持续开火 为节约子弹 则强制性停止开火 
          stop_scopperil_fire = 0;
          fire = false;
          which_if = 55;
        }
        // std::cout << "solve_yaw1: " << solve_yaw1 << " " << solve_yaw2 << " " << solve_yaw3 << " " << pitch_w << std::endl;
        // std::cout << "shoot_yaw: " << shoot_yaw << " shoot_pitch: " << shoot_pitch << std::endl;
        // std::cout << "111solve_xa1: " << solve_xa1 << " solve_ya1: " << solve_ya1 << std::endl;
        // std::cout << "111solve_xa2: " << solve_xa2 << " solve_ya2: " << solve_ya2 << std::endl;
        // std::cout << "111solve_xa3: " << solve_xa3 << " solve_ya3: " << solve_ya3 << std::endl;
        // std::cout << "111solve_za: " << solve_za << std::endl;
      }
    } else {
      // std::cout << "   solve_xa1: " << solve_xa1 << " solve_ya1: " << solve_ya1 << " solve_zc: " << solve_zc << std::endl;

      // point_in_move_link.x = solve_xc;
      // point_in_move_link.y = solve_yc;
      // point_in_move_link.z = solve_zc;
      // point_in_gun_link = transformPoint(point_in_move_link, "gun_link", "move_link");
      // solve_xc = point_in_gun_link.x;
      // solve_yc = point_in_gun_link.y;

      point_in_move_link.x = solve_xa1;
      point_in_move_link.y = solve_ya1;
      point_in_move_link.z = solve_zc;
      point_in_gun_link = transformPoint(point_in_move_link, "gun_link", "move_link");
      solve_xa1 = point_in_gun_link.x;
      solve_ya1 = point_in_gun_link.y;
      solve_zc = point_in_gun_link.z;
      // std::cout << "111solve_xa1: " << solve_xa1 << " solve_ya1: " << solve_ya1 << " solve_zc: " << solve_zc << std::endl;

      pitch_w = atan2(solve_zc, solve_xa1);

      // 非小陀螺则选择当前装甲板
      solve_xc = solve_xa1;
      solve_yc = solve_ya1;

      which_if = 66;
      last_yaw += scopperil_predict_dt * solve_v_yaw;
      solve_last_x = (last_x - solve_r1 * cos(last_yaw)) + t_flight * solve_vx;
      solve_last_y = (last_y - solve_r1 * sin(last_yaw)) + t_flight * solve_vy;

      point_in_move_link.x = solve_last_x;
      point_in_move_link.y = solve_last_y;
      // point_in_move_link.z = solve_zc;
      point_in_gun_link = transformPoint(point_in_move_link, "gun_link", "move_link");
      solve_last_x = point_in_gun_link.x;
      solve_last_y = point_in_gun_link.y;
      // solve_zc = point_in_gun_link.z;
      
      last_yaw_w = -atan2(solve_last_y, solve_last_x);

      // 非小陀螺状态下满足条件开火
      if (abs(last_yaw_w) < shoot_yaw && abs(pitch_w) < shoot_pitch) {
        fire = true;
      }
    }

    // 计算相机分别移动的到三块装甲板需要转动的角度，弧度制
    yaw_w = -atan2(solve_yc, solve_xc);


    // 使用二分法进行弹道解算 并计算子弹飞行时间
    max_theta = 0.25 * sove_PI, min_theta = -0.16 * sove_PI, x1 = solve_xa1, y1 = solve_zc + dis_z;
    for (int i = 0; i < 10; i ++) {
      theta = (max_theta + min_theta) / 2;
      max_value = SolveTrajectory(max_theta, x1);
      min_value = SolveTrajectory(min_theta, x1);
      mid_value = SolveTrajectory(theta, x1);
      if ((max_value - y1) * (mid_value - y1) > 0){
        max_theta = theta;
      } else if ((min_value - y1) * (mid_value - y1) > 0){
        min_theta = theta;
      }
    }
    theta =  (max_theta + min_theta) / 2;
    pitch_w += theta;
    double term1 = (k * solve_xc) / (m * v * cos(theta));
    if (term1 >= 1) {
      // std::cout << "解析法：无法计算时间，x1超过最大射程" << std::endl;
      t_flight = 0.5;
    }
    else {
      t_flight = -(m / k) * std::log(1 - term1);
      // std::cout << "飞行时间:" << t_flight << std::endl;
    }
    // t_flight = 0.0;


    //更新预测时间
    scopperil_predict_dt_ = scopperil_predict_dt + t_flight;
    move_predict_dt_ = move_predict_dt + t_flight;
    // std::cout << "scopperil_predict_dt_:" << scopperil_predict_dt_ << std::endl;
    // std::cout << "move_predict_dt_:" << move_predict_dt_ << std::endl;
    // std::cout << "solve_v_yaw:" << solve_v_yaw << std::endl;
    // std::cout << "change_pitch2:" << pitch_w << std::endl;
    // std::cout << "current_yaw:" << current_yaw << std::endl;
    // std::cout << "fire:" << fire << std::endl;
    // std::cout << "is_scopperil:" << is_scopperil << std::endl;


    // 发布消息
    trajectory_msg.header.stamp = rclcpp::Clock().now();
    trajectory_msg.header.frame_id = target_frame_;
    if (gimbal_rotate_) {
      if (std::to_string(abs(yaw_w)) == "nan"){
        yaw_w = 0;
      }
      if (std::to_string(abs(pitch_w)) == "nan"){
        pitch_w = 0;
      }
      trajectory_msg.yaw_w = yaw_w;
      trajectory_msg.pitch_w = pitch_w;
      trajectory_msg.current_yaw = current_yaw;
      trajectory_msg.fire = fire;
      trajectory_msg.need_navigation = 0;
      // trajectory_msg.fire = 1;
      trajectory_msg.tracking = solve_tracking;
    } else {
      trajectory_msg.yaw_w = 0;
      trajectory_msg.pitch_w = 0;
      trajectory_msg.current_yaw = 0;
      trajectory_msg.fire = 0;
      trajectory_msg.tracking = 1;
      trajectory_msg.need_navigation = 1;
    }
    trajectory_pub_->publish(trajectory_msg);

  } else {
    trajectory_msg.header.stamp = rclcpp::Clock().now();
    trajectory_msg.header.frame_id = target_frame_;
    trajectory_msg.yaw_w = 0.0;
    trajectory_msg.pitch_w = 0.0;
    trajectory_msg.current_yaw = current_yaw;
    trajectory_msg.fire = 0;
    trajectory_msg.tracking = (720 - last_u > 0) ? 0 : 2;
    trajectory_msg.need_navigation = 0;
    trajectory_pub_->publish(trajectory_msg);
  }
}
}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::TrajectoryNode)
