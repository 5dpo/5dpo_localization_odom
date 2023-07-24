# [5dpo_localization_odom](https://github.com/5dpo/5dpo_localization_odom)

This repository implements a data processor for the estimation of the robot pose
based on odometric-only data (e.g., wheeled, laser, visual, and/or inertial
odometry). The ROS package implemented in this repository makes available nodes
that may leverage one and/or multiple sources of odometry data for dead
reckoning pose estimation of the robot.

**Version 1.4.0**

**With this version, it is possible to do:**

- Wheeled odometry (four-wheeled omnidirectional steering geometry)
- Wheeled odometry (differential and three-wheeled omnidirectional steering
  geometries)
- Inverse wheeled odometry
- Enable/disable publication of the tf base_frame_id > odom_frame_id
- Limit the maximum angular speed of the wheels and scale the velocities

**The next version will add these features:**

- Wheeled odometry (tricycle steering geometry)
- Fusion of wheels and inertial odometry data

## ROS

**foxy**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS 2 Foxy](https://docs.ros.org/en/foxy/)

**noetic**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS 1 Noetic](https://wiki.ros.org/noetic/)

### Dependencies

- [rclcpp](https://index.ros.org/r/rclcpp/)
- [geometry_msgs](https://index.ros.org/p/geometry_msgs/)
- [nav_msgs](https://index.ros.org/p/nav_msgs/)
- [sdpo_drivers_interfaces](https://github.com/5dpo/5dpo_drivers_interfaces)
- [tf2](https://index.ros.org/p/tf2/)
- [tf2_ros](https://index.ros.org/p/tf2_ros/)

### Parameters

- base_frame_id (`std::string = "base_footprint"`): tf frame id of the robot
  base footprint coordinate frame
- odom_frame_id (`std::string = "odom"`): tf frame id of the robot odometry
  coordinate frame
- publish_tf (`bool = true`): enable the publication of the tf
  base_frame_id > odom_frame_id
- invert_tf (`bool = false`): enable the invertion of the tf (odom_frame_id >
  base_frame_id instead of base_frame_id > odom_frame_id)
- w_ref_max_enabled (`bool = false`): enable a maximum angular speed for the
  robot wheels
- w_ref_max (`double`): maximum angular speed of the robot wheels
- steering_geometry: steering geometry of the mobile robot (rad/s)
  (`"diff" | "tricyc" | "omni3" | "omni4"`)
  - Differential drive
    - rob_dist_between_wh (`double`): distance between left-right wheels (m)
    - wh_right_diam (`double`): right wheel diameter (m)
    - wh_right_idx (`size_t`): right wheel index in the
      [mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg)
      (0..1)
    - wh_right_inv (`bool`): invert positive direction of the right
      wheel angular motion
    - wh_left_diam (`double`): left wheel diameter (m)
    - wh_left_idx (`size_t`): left wheel index in the
      [mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg)
      (0..1)
    - wh_left_inv (`bool`): invert positive direction of the left
      wheel angular motion
  - Tricycle
    - TBD 
  - Three-wheeled omnidirectional robot
    - rob_dist_center_wh (`double`): distance between the robot geometric center
      and the omnidirectional wheels (m)
    - wh_front_right_diam (`double`): front-right wheel diameter (m)
    - wh_front_right_idx (`size_t`): front-right wheel index in the
      [mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg)
      (0..2)
    - wh_front_right_inv (`bool`): invert positive direction of the front-right
      wheel angular motion
    - wh_front_left_diam (`double`): front-left wheel diameter (m)
    - wh_front_left_idx (`size_t`): front-left wheel index in the
      [mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg)
      (0..2)
    - wh_front_left_inv (`bool`): invert positive direction of the front-left
      wheel angular motion
    - wh_back_diam (`double`): back wheel diameter (m)
    - wh_back_idx (`size_t`): back wheel index in the
      [mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg)
      (0..2)
    - wh_back_inv (`bool`): invert positive direction of the back
      wheel angular motion
  - Four-wheeled omnidirectional robot
    - rob_dist_between_front_back_wh (`double`): distance between front-back
      wheels (m)
    - rob_dist_between_left_right_wh (`double`): distance between left-right
      wheels (m)
    - wh_front_left_diam (`double`): front-left wheel diameter (m)
    - wh_front_left_idx (`size_t`): front-left wheel index in the
      [mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg)
      (0..3)
    - wh_front_left_inv (`bool`): invert positive direction of the front-left
      wheel angular motion
    - wh_front_right_diam (`double`): front-right wheel diameter (m)
    - wh_front_right_idx (`size_t`): front-right wheel index in the
      [mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg)
      (0..3)
    - wh_front_right_inv (`bool`): invert positive direction of the front-right
      wheel angular motion
    - wh_back_left_diam (`double`): back-left wheel diameter (m)
    - wh_back_left_idx (`size_t`): back-left wheel index in the
      [mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg)
      (0..3)
    - wh_back_left_inv (`bool`): invert positive direction of the back-left
      wheel angular motion
    - wh_back_right_diam (`double`): back-right wheel diameter (m)
    - wh_back_right_idx (`size_t`): back-right wheel index in the
      [mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg)
      (0..3)
    - wh_back_right_inv (`bool`): invert positive direction of the back-right
      wheel angular motion

### Subscribes

- cmd_vel
  ([Twist.msg](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html))
- motors_enc
  ([MotEncArray.msg](https://github.com/5dpo/5dpo_drivers_interfaces/blob/foxy/msg/MotEncArray.msg))

### Publishes

- cmd_vel_ref
  ([Twist.msg](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html))
- motors_ref
  ([MotRefArray.msg](https://github.com/5dpo/5dpo_drivers_interfaces/blob/foxy/msg/MotRefArray.msg))
- odom
  ([Odometry.msg](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html))
- tf (N/A)
  - base_frame_id > odom_frame_id

### Services

None.

### Actions

None.

## Usage

### Build

```sh
# ROS 2
source /opt/ros/foxy/setup.bash

# Create workspace
mkdir -p ~/ros2_ws/src

# Clone the repository
cd ~/ros2_ws/src
git clone git@github.com:5dpo/5dpo_localization_odom.git

# Build
colcon build
source install/setup.bash
```

### Launch

**sdpo_localization_odom_wh**

```sh
ros2 launch sdpo_localization_odom sdpo_localization_odom_wh.launch.xml
```

## Acknowledges

- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)

## Contacts

If you have any questions or you want to know more about this work, please
contact any member of the 5dpo Robotics Team.
