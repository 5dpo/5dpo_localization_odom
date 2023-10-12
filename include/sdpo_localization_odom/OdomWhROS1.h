#pragma once

#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sdpo_drivers_interfaces/MotEncArrayROS1.h>
#include <sdpo_drivers_interfaces/MotRefArrayROS1.h>
#include <tf2_ros/transform_broadcaster.h>

#include "sdpo_localization_odom/OdomWh.h"

namespace sdpo_localization_odom {

class OdomWhROS1
{

 private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Publisher pub_cmd_vel_ref_;
  ros::Publisher pub_mot_ref_;
  ros::Publisher pub_odom_;

  ros::Subscriber sub_mot_enc_;
  ros::Subscriber sub_cmd_vel_;



  tf2_ros::TransformBroadcaster tf_broad_;



  std::unique_ptr<OdomWh> odom_;



  std::string base_frame_id_;
  std::string odom_frame_id_;
  bool publish_tf_;
  bool invert_tf_;
  std::string steering_geometry_;
  bool w_ref_max_enabled_;
  double w_ref_max_;





 public:

  OdomWhROS1();

  ~OdomWhROS1() = default;



 private:

  void readParam();

  void subMotEnc(
      const sdpo_drivers_interfaces::MotEncArrayROS1::ConstPtr& msg);
  void subCmdVel(
      const geometry_msgs::Twist::ConstPtr& msg);

  void pubCmdVelRef();
};

} // namespace sdpo_localization_odom
