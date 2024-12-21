# walk

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_humble.yaml?query=branch:iron)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_iron.yaml?query=branch:iron)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

ROS2 Walking Node to be used across different types of bipeds

# Compiling

In your workspace, run:
```
git clone git@github.com:ijnek/walk.git src/walk
vcs import src < src/walk/dependencies.repos --recursive
colcon build
```

# Instructions, all in separate terminals

Start nao_lola_client on the robot:
```
ros2 run nao_lola_client nao_lola_client
```

Stiffen all joints on the robot
```
ros2 topic pub --once /effectors/joint_stiffnesses nao_lola_command_msgs/msg/JointStiffnesses "{indexes: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24], stiffnesses: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
```

Start inverse kinematics on the robot:
```
ros2 run nao_ik nao_ik
```

Start the phase provider on the robot:
```
ros2 run nao_phase_provider nao_phase_provider --ros-args -r fsr:=/sensors/fsr
```

Make robot crouch with (on your computer):
```
ros2 topic pub --once /motion/sole_poses biped_interfaces/msg/SolePoses "
l_sole:
  position:
    y: 0.05
    z: -0.315
r_sole:
  position:
    y: -0.05
    z: -0.315
"
```

Start the walk node on the robot:
```
ros2 run walk walk
```

Start teleop on your computer:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=target
```

Start debugging with rqt (on your computer) using an layout for debugging the walk:
```
rqt --perspective-file $(ros2 pkg prefix --share walk)/perspective/walk/walk.perspective
```

### Additional Instructions

To abort the walk immediately:
```
ros2 service call /abort std_srvs/srv/Empty
```

To limp the robot:
```
ros2 topic pub --once /effectors/joint_stiffnesses nao_lola_command_msgs/msg/JointStiffnesses "{indexes: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24], stiffnesses: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```
