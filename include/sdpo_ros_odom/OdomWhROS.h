#pragma once

#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sdpo_ros_interfaces_hw/mot_enc_array.h>
#include <sdpo_ros_interfaces_hw/mot_ref.h>
#include <tf/transform_broadcaster.h>

#include "sdpo_ros_odom/OdomWh.h"

namespace sdpo_ros_odom {

class OdomWhROS {
 public:
  static const std::string kOdomWhTypeOmni4Str;

 private:
  ros::NodeHandle nh;

  ros::Publisher pub_mot_ref_;
  ros::Publisher pub_odom_;
  ros::Subscriber sub_mot_enc_;
  ros::Subscriber sub_cmd_vel_;

  tf::TransformBroadcaster tf_broad_;

  std::unique_ptr<OdomWh> odom_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  std::string steering_geometry_;

 public:
  OdomWhROS();
  ~OdomWhROS() = default;

 private:
  bool readParam();

  void subMotEnc(const sdpo_ros_interfaces_hw::mot_enc_array& msg);
  void subCmdVel(const geometry_msgs::Twist& msg);
};

} // namespace sdpo_ros_odom
