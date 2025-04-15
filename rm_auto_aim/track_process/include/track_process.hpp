// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef RM_TRACK_PROCESS__TRACK_PROCESS_HPP_
#define RM_TRACK_PROCESS__TRACK_PROCESS_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
// #include <std_msgs/msg/float64.hpp>
// #include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/core.hpp>
// #include <opencv2/core/types.hpp>
#include <opencv2/videoio.hpp>

// C++ system
// #include <future>
// #include <memory>
#include <string>
#include <thread>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/trajectory.hpp"
#include "auto_aim_interfaces/msg/is_reach_goal.hpp"



namespace rm_auto_aim
{
class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode(const rclcpp::NodeOptions & options);

// ///////////////////////////////////////////////////////////////////////////////////////// 快速注释线
//   explicit TrajectoryNode(const rclcpp::NodeOptions & options);
//   ~TrajectoryNode() override;
//   ////////////////////////////////////////////////////////////////////////////////////// 快速注释线

private:
  double SolveTrajectory(double theta, double x);
  void trackCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void GimbalRotateCallback(const auto_aim_interfaces::msg::IsReachGoal::SharedPtr msg);
  void _img_sub_();
  void _odemetry_sub_();
  void _gimbal_rotate_sub_();

  geometry_msgs::msg::Point transformPoint(geometry_msgs::msg::Point point_in_move_link,
  std::string point_in_target, std::string point_in_original);

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::vector<double> v_yaw_vec, dz_vec, vx_vec, vy_vec;

  std::string target_frame_;

  std::thread odemetry_thread_;
  std::thread img_thread_;
  std::thread gimbal_rotate_thread_;

  double m, d, Cd, rho, g, v, dis_x, dis_z, sove_PI;
  double r, A, k;
  double theta, max_value, min_value, mid_value;
  double last_x, last_y, last_z, last_yaw;
  double pitch_w, yaw_w, max_theta, min_theta, x1, y1, current_roll, current_pitch, current_yaw, scopperil_seconds;
  double solve_xc, solve_yc, solve_zc, solve_yaw, solve_vx, solve_vy, solve_v_yaw, solve_r1, solve_r2, solve_dz;
  double solve_za, solve_xa1, solve_ya1, solve_xa2, solve_ya2, solve_yaw1, solve_yaw2, solve_xa3, solve_ya3, solve_yaw3;
  double scopperil_predict_dt, move_predict_dt, last_receive_reach_goal_time;
  int fire, stop_scopperil_fire, scopperil_sount, solve_armors_num, turn_right, which_if;
  float one_angle, solve_dis, shoot_yaw, shoot_pitch, u_, v_, detect_yaw_, last_u;
  float scopperil_predict_dt_, move_predict_dt_, t_flight, solve_last_x, solve_last_y, last_yaw_w;
  bool should_zero, is_scopperil, solve_tracking, gimbal_rotate_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr track_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::IsReachGoal>::SharedPtr gimbal_rotate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odemetry_sub_;

  rclcpp::Publisher<auto_aim_interfaces::msg::Trajectory>::SharedPtr trajectory_pub_;

  rclcpp::Time last_time;

  auto_aim_interfaces::msg::Trajectory trajectory_msg;

  image_transport::Publisher draw_img_pub_;

  cv::Mat camera_matrix, distortion_coeffs, rvec, tvec, rgbFrame;
  cv::VideoWriter out;
};

}

#endif  // RM_TRACK_PROCESS__TRACK_PROCESS_HPP_