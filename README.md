# walk

[![Build and Test (foxy)](https://github.com/ijnek/walk/actions/workflows/build_and_test_foxy.yaml/badge.svg)](https://github.com/ijnek/walk/actions/workflows/build_and_test_foxy.yaml)
[![Build and Test (galactic)](https://github.com/ijnek/walk/actions/workflows/build_and_test_galactic.yaml/badge.svg)](https://github.com/ijnek/walk/actions/workflows/build_and_test_galactic.yaml)
[![Build and Test (rolling)](https://github.com/ijnek/walk/actions/workflows/build_and_test_rolling.yaml/badge.svg)](https://github.com/ijnek/walk/actions/workflows/build_and_test_rolling.yaml)

ROS2 Walking Node

## This package is not ready for usage**

# Instructions, all in separate terminals

Start simulator:
```
rcsoccersim3d
```

Launch robot in simulator:
```
ros2 run rcss3d_nao rcss3d_nao
```

Start inverse kinematics of robot:
```
ros2 run nao_ik ik_node 
```

Make robot crouch with:
```
ros2 topic pub --once /motion/ik_command nao_ik_interfaces/msg/IKCommand "left_ankle:
  position:
    x: 0.0
    y: 0.05
    z: -0.18
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
right_ankle:
  position:
    x: 0.0
    y: -0.05
    z: -0.18
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
"
```

Start the walk node:
```
ros2 run walk walk
```

Start teleop to control the robot:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=target
```

### Additional Instructions

To abort the walk immediately:
```
ros2 service call /abort std_srvs/srv/Empty
```