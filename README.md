# 5dpo_ros_odom

**Version 0.0.0**

This repository implements a data processor for the estimation of the robot pose
based on odometric-only data (e.g., wheeled, laser, visual, and/or inertial
odometry). The ROS package implemented in this repository makes available nodes
that may leverage one and/or multiple sources of odometry data for dead
reckoning pose estimation of the robot.

**With this version, it is possible to do:**

- TBD

**The next version will add these features:**

- Wheeled odometry
- Fusion of wheels and inertial odometry data

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

### Parameters

**sdpo_ros_odom_wh**

- TBD

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

TBC

### Launch

TBC

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
