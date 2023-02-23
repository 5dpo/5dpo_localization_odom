#include "sdpo_ros_odom/OdomWhROS.h"

#include <exception>

namespace sdpo_ros_odom {

static const std::string kOdomWhTypeOmni4Str = "omni4";

OdomWhROS::OdomWhROS() {
  try {
    readParam();
  } catch (std::exception& e) {
    ROS_FATAL("[sdpo_ros_odom] Error reading the node parameters (%s)",
              e.what());
    ros::shutdown();
  }

  pub_mot_ref_ = nh.advertise<sdpo_ros_interfaces_hw::mot_ref>("motors_ref", 1);
  pub_odom_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
  sub_mot_enc_ = nh.subscribe("motors_encoders", 1,
                              &OdomWhROS::subMotEnc, this);
  sub_cmd_vel_ = nh.subscribe("cmd_vel", 1,
                              &OdomWhROS::subCmdVel, this);
}

bool OdomWhROS::readParam() {
  // TODO: put vector of bool for invert motion in motors!!!

  return true;
};

void OdomWhROS::subMotEnc(const sdpo_ros_interfaces_hw::mot_enc_array& msg) {
  /*try {
    for(int i = 0; i < msg.mot_enc_array_data.size(); i++) {
      odom_->setMotorDriveEncTicksDelta(
          i, msg.mot_enc_array_data[i].encoder_delta,
          msg.mot_enc_array_data[i].ticks_per_rev);
      odom_->setMotorDriveW(i, msg.mot_enc_array_data[i].angular_speed);
    }

    odom_->update();

    geometry_msgs::Twist odom_vel;
    odom_vel.linear.x = odom_->vel.v;
    odom_vel.linear.y = odom_->vel.vn;
    odom_vel.linear.z = 0.0;
    odom_vel.angular.x = 0.0;
    odom_vel.angular.y = 0.0;
    odom_vel.angular.z = odom_->vel.w;

    tf::StampedTransform odom2base_tf;
    odom2base_tf.setOrigin(tf::Vector3(odom_->pose.x, odom_->pose.y, 0.0));
    odom2base_tf.setRotation(tf::createQuaternionFromYaw(odom_->pose.th));
    odom2base_tf.stamp_ = msg.stamp;
    odom2base_tf.frame_id_ = "odom";
    odom2base_tf.child_frame_id_ = "base";
    tf_broad_.sendTransform(odom2base_tf);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = odom2base_tf.stamp_;
    odom_msg.header.frame_id = odom2base_tf.frame_id_;
    odom_msg.child_frame_id = odom2base_tf.child_frame_id_;
    odom_msg.pose.pose.position.x = odom2base_tf.getOrigin().x();
    odom_msg.pose.pose.position.y = odom2base_tf.getOrigin().y();
    odom_msg.pose.pose.position.z = odom2base_tf.getOrigin().z();
    odom_msg.pose.pose.orientation.x = odom2base_tf.getRotation().x();
    odom_msg.pose.pose.orientation.y = odom2base_tf.getRotation().y();
    odom_msg.pose.pose.orientation.z = odom2base_tf.getRotation().z();
    odom_msg.pose.pose.orientation.w = odom2base_tf.getRotation().w();
    odom_msg.twist.twist = odom_vel;

    pub_odom_.publish(odom_msg);

  } catch (std::exception& e) {
    ROS_ERROR("[sdpo_ros_odom] Error when processing the motor encoders data "
              "message (%s)", e.what());
  }*/
}

void OdomWhROS::subCmdVel(const geometry_msgs::Twist& msg) {
  /*odom_->setVelRef(msg.linear.x, msg.linear.y, msg.angular.z);

  sdpo_ros_interfaces_hw::mot_ref mot_ref_msg;
  mot_ref_msg.angular_speed_ref.resize(odom_->mot.size());
  for(int i = 0; i < odom_->mot.size(); i++) {
    mot_ref_msg.angular_speed_ref[i] = odom_->getMotorDriveWr(i);
  }*/
}

} // namespace sdpo_ros_odom
