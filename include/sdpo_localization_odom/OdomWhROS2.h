#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdpo_drivers_interfaces/msg/mot_enc_array.hpp>
#include <sdpo_drivers_interfaces/msg/mot_ref_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "sdpo_localization_odom/OdomWh.h"

namespace sdpo_localization_odom {

class OdomWhROS2 : public rclcpp::Node
{

 private:

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_ref_;

  rclcpp::Publisher<sdpo_drivers_interfaces::msg::MotRefArray>::SharedPtr
      pub_mot_ref_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;



  rclcpp::Subscription<sdpo_drivers_interfaces::msg::MotEncArray>::SharedPtr
      sub_mot_enc_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;



  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broad_;



  std::unique_ptr<OdomWh> odom_;



  std::string base_frame_id_;

  std::string odom_frame_id_;

  bool publish_tf_;
  bool invert_tf_;

  std::string steering_geometry_;

  bool w_ref_max_enabled_;

  double w_ref_max_;





 public:

  OdomWhROS2();

  ~OdomWhROS2() = default;



 private:

  void readParam();

  void subMotEnc(
      const sdpo_drivers_interfaces::msg::MotEncArray::SharedPtr msg);
  void subCmdVel(
      const geometry_msgs::msg::Twist::SharedPtr msg);

  void pubCmdVelRef();
};

} // namespace sdpo_localization_odom
