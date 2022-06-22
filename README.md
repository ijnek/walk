# walk

[![Build and Test (rolling)](https://github.com/ijnek/walk/actions/workflows/build_and_test_rolling.yaml/badge.svg)](https://github.com/ijnek/walk/actions/workflows/build_and_test_rolling.yaml)

ROS2 Walking Node to be used across different types of bipeds

## This package is not ready for usage

# Compiling

In your workspace, run:
```
git clone git@github.com:ijnek/walk.git src/walk
vcs import src < src/walk/dependencies.repos --recursive
colcon build
```

# Instructions, all in separate terminals

Start simulator:
```
rcsoccersim3d
```

Launch NAO in simulator:
```
ros2 run rcss3d_nao rcss3d_nao
```

Start inverse kinematics of robot:
```
ros2 run nao_ik ik_node
```

Start the phase provider:
```
ros2 run nao_phase_provider nao_phase_provider --ros-args -r fsr:=/sensors/fsr
```

Make robot crouch with:
```
ros2 topic pub --once /motion/sole_poses biped_interfaces/msg/SolePoses "
l_sole:
  position:
    y: 0.05
    z: -0.23
r_sole:
  position:
    y: -0.05
    z: -0.23
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

Start debugging with rqt using an layout for debugging the walk:
```
rqt --perspective-file $(ros2 pkg prefix walk)/perspective/walk/walk.perspective
```

### Additional Instructions

To abort the walk immediately:
```
ros2 service call /abort std_srvs/srv/Empty
```
