#!/bin/zsh

source ./install/setup.sh

ros2 topic pub /tracker/target auto_aim_interfaces/msg/Target "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: ''
  },
  tracking: false,
  id: '',
  armors_num: 1,
  position: {x: 5.0, y: -3.0, z: 0.0},
  yaw: 0.0,
  v_yaw: 0.0,
  radius_1: 0.0,
  radius_2: 0.0,
  dz9: 0.0
}"