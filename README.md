# 5dpo_ros_odom

**Version 1.1.1**

This repository implements a data processor for the estimation of the robot pose
based on odometric-only data (e.g., wheeled, laser, visual, and/or inertial
odometry). The ROS package implemented in this repository makes available nodes
that may leverage one and/or multiple sources of odometry data for dead
reckoning pose estimation of the robot.

**With this version, it is possible to do:**

- Wheeled odometry (four-wheeled omnidirectional steering geometry)
- Wheeled odometry (differential and three-wheeled omnidirectional steering
  geometries)

**The next version will add these features:**

- Wheeled odometry (tricycle steering geometry)
- Limit the maximum angular speed of the wheels and scale the velocities
- Fusion of wheels and inertial odometry data

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [geometry_msgs](https://wiki.ros.org/geometry_msgs)
- [nav_msgs](https://wiki.ros.org/nav_msgs)
- [sdpo_ros_interfaces_hw](https://github.com/5dpo/5dpo_ros_interfaces)
- [tf](https://wiki.ros.org/tf)

### Parameters

- base_frame_id (`std::string = "base_footprint"`): tf frame id of the robot
  base footprint coordinate frame
- odom_frame_id (`std::string = "odom"`): tf frame id of the robot odometry
  coordinate frame
- steering_geometry: steering geometry of the mobile robot
  (`"diff" | "tricyc" | "omni3" | "omni4"`)
  - w_ref_max_enabled (`bool = false`): enable a maximum angular speed for the
    robot wheels
  - w_ref_max (`double`): maximum angular speed of the robot wheels
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
  ([Twist.msg](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

**sdpo_ros_odom_wh**

- motors_encoders
  ([mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg))

### Publishes

- motors_ref
  ([mot_ref.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_ref.msg))
- odom
  ([Odometry.msg](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

### Services

None.

### Actions

None.

## Usage

### Build

```sh
# Create catkin workspace
mkdir -p ~/catkin_ws/src

# Clone repository
cd ~/catkin_ws/src
git clone git@github.com:5dpo/5dpo_ros_odom.git

# Build
cd ..
catkin build
```

### Launch

**sdpo_ros_odom_wh**

```sh
roslaunch sdpo_ros_odom sdpo_ros_odom_wh.launch
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Héber Miguel Sobreira ([gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [inesctec](mailto:heber.m.sobreira@inesctec.pt))
- João G. Martins ([github](https://github.com/Joao-G-Martins),
  [feup](mailto:up201806222@edu.fe.up.pt),
  [inesctec](mailto:joao.g.martins@inesctec.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
