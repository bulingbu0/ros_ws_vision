// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
/*############################################################*/
// vision
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  uint8_t task_mode : 2;     // 0-auto 1-aim 2-buff
  bool reset_tracker : 1;
  uint8_t is_play : 1;
  bool change_target : 1;
  uint8_t reserved : 2;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t game_time;  // (s) game time [0, 450]
  uint32_t timestamp;  // (ms) board time
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t state;       // 0-untracking 1-tracking-aim 2-tracking-buff
  uint8_t track;
  // uint8_t id : 3;          // aim: 0-outpost 6-guard 7-base
  // uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  // float x;                 // aim: robot-center || buff: rune-center
  // float y;                 // aim: robot-center || buff: rune-center
  // float z;                 // aim: robot-center || buff: rune-center
  // float yaw;               // aim: robot-yaw || buff: rune-theta
  // // spd = a*sin(w*t)+b || spd > 0 ==> clockwise
  // float vx;  // aim: robot-vx || buff: rune spin speed param - a
  // float vy;  // aim: robot-vy || buff: rune spin speed param - b
  // float vz;  // aim: robot-vz || buff: rune spin speed param - w
  // float v_yaw;
  // float r1;
  // float r2;
  // float dz;
  // uint32_t cap_timestamp;  // (ms) frame capture time
  // uint16_t t_offset;       // (ms) speed t offset
  float change_pitch;
  float change_yaw;

  //ROS
  float speed_x;
  float speed_y;
  float angular;
  uint8_t spin_flag;
  uint16_t checksum = 0;
} __attribute__((packed));


/*############################################################*/
// ROS
struct ReceiveRobotMotionData
{
  uint8_t head;
  uint8_t spin_flag;

  // 比赛信息数据
  uint8_t game_progress;
  uint32_t stage_remain_time; //当前阶段剩余时间，单位s

  // 事件数据
  // uint8_t supply_station_front;
  // uint8_t supply_station_internal;
  // uint8_t supply_zone;
  // uint8_t center_gain_zone;
  // uint8_t small_energy;
  // uint8_t big_energy;
  // uint8_t circular_highland;
  // uint8_t trapezoidal_highland_3;
  // uint8_t trapezoidal_highland_4;
  // uint8_t base_virtual_shield_remaining;

  // 全场机器人hp信息数据
    // uint16_t red_1_robot_hp;
    // uint16_t red_2_robot_hp;
    // uint16_t red_3_robot_hp;
    // uint16_t red_4_robot_hp;
    // uint16_t red_5_robot_hp;
    // uint16_t red_7_robot_hp;
    // uint16_t red_outpost_hp;
    // uint16_t red_base_hp;
    // uint16_t blue_1_robot_hp;
    // uint16_t blue_2_robot_hp;
    // uint16_t blue_3_robot_hp;
    // uint16_t blue_4_robot_hp;
    // uint16_t blue_5_robot_hp;
    // uint16_t blue_7_robot_hp;
    // uint16_t blue_outpost_hp;
    // uint16_t blue_base_hp;

    // 地面机器人位置数据
    // float hero_x;
    // float hero_y;

    // float engineer_x;
    // float engineer_y;

    // float standard_3_x;
    // float standard_3_y;

    // float standard_4_x;
    // float standard_4_y;

    // float standard_5_x;
    // float standard_5_y;

    // RFID 状态数据包
    // bool base_gain_point;                                // 己方基地增益点
    // bool central_highland_gain_point;;                   // 己方中央高地增益点
    // bool enemy_central_highland_gain_point;              // 对方中央高地增益点
    // bool friendly_trapezoidal_highland_gain_point;       // 己方梯形高地增益点
    // bool enemy_trapezoidal_highland_gain_point;          // 对方梯形高地增益点
    // bool friendly_fly_ramp_front_gain_point;             // 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
    // bool friendly_fly_ramp_back_gain_point;              // 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
    // bool enemy_fly_ramp_front_gain_point;                // 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
    // bool enemy_fly_ramp_back_gain_point;                 // 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
    // bool friendly_central_highland_lower_gain_point;     // 己方地形跨越增益点（中央高地下方）
    // bool friendly_central_highland_upper_gain_point;     // 己方地形跨越增益点（中央高地上方）
    // bool enemy_central_highland_lower_gain_point;        // 对方地形跨越增益点（中央高地下方）
    // bool enemy_central_highland_upper_gain_point;        // 对方地形跨越增益点（中央高地上方）
    // bool friendly_highway_lower_gain_point;              // 己方地形跨越增益点（公路下方）
    // bool friendly_highway_upper_gain_point;              // 己方地形跨越增益点（公路上方）
    // bool enemy_highway_lower_gain_point;                 // 对方地形跨越增益点（公路下方）
    // bool enemy_highway_upper_gain_point;                 // 对方地形跨越增益点（公路上方）
    bool friendly_fortress_gain_point;                   // 己方堡垒增益点
    // bool friendly_outpost_gain_point;                    // 己方前哨站增益点
    bool friendly_supply_zone_non_exchange;              // 己方与兑换区不重叠的补给区/RMUL 补给区
    bool friendly_supply_zone_exchange;                  // 己方与兑换区重叠的补给区
    // bool friendly_big_resource_island;                   // 己方大资源岛增益点
    // bool enemy_big_resource_island;                      // 对方大资源岛增益点
    bool center_gain_point;                              // 中心增益点（仅 RMUL 适用）

    // 机器人状态数据
    // uint8_t robot_id;
    // uint8_t robot_level;
    uint16_t current_hp; // 当前血量
    // uint8_t ARMOR_HIT;
    // bool is_hp_deduced; // 血量是否减少
    uint16_t projectile_allowance_17mm;
    // uint16_t maximum_hp;
    // uint16_t shooter_barrel_cooling_value;
    // uint16_t shooter_barrel_heat_limit;

    uint16_t shooter_17mm_1_barrel_heat;

    uint8_t armor_id;
    uint8_t hp_deduction_reason; //血量减少的原因

    // 云台状态数据
    // float pitch;
    // float yaw;

    // uint16_t projectile_allowance_17mm_1;
    // uint16_t remaining_gold_coin;
    uint16_t crc;
} __attribute__((packed));

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_