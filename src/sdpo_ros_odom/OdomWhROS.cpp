#include "sdpo_ros_odom/OdomWhROS.h"

#include <exception>
#include <vector>

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
  ros::NodeHandle nh_private("~");

  if (!nh_private.hasParam("steering_geometry")) {
    throw std::runtime_error(
        "[OdomWhROS.cpp] OdomWhROS::readParam: "
        "the node sdpo_ros_odom requires the definition of the "
        "steering_geometry parameter");
  }

  auto print_is_default_param_set =
      [&nh_private](const std::string& param_name) {
        if (!nh_private.hasParam(param_name)) {
          ROS_INFO("[sdpo_ros_odom] Parameter %s not set in the parameter "
                   "server (using default value)",
                   param_name.c_str());
        }
      };

  print_is_default_param_set("base_frame_id");
  nh_private.param<std::string>("base_frame_id", base_frame_id_,
                                "base_footprint");
  ROS_INFO("[sdpo_ros_odom] Base frame ID: %s", base_frame_id_.c_str());

  print_is_default_param_set("odom_frame_id");
  nh_private.param<std::string>("odom_frame_id", odom_frame_id_,
                                "odom");
  ROS_INFO("[sdpo_ros_odom] Odom frame ID: %s", odom_frame_id_.c_str());

  nh_private.getParam("steering_geometry", steering_geometry_);
  ROS_INFO("[sdpo_ros_odom] Steering geometry: %s", steering_geometry_.c_str());

  if (steering_geometry_ == kOdomWhOmni4Str) {
    if (!nh_private.hasParam("rob_dist_between_front_back_wh") ||
            !nh_private.hasParam("rob_dist_between_left_right_wh") ||
        !nh_private.hasParam("wh_front_left_diam") ||
            !nh_private.hasParam("wh_front_left_idx") ||
            !nh_private.hasParam("wh_front_left_inv") ||
        !nh_private.hasParam("wh_front_right_diam") ||
            !nh_private.hasParam("wh_front_right_idx") ||
            !nh_private.hasParam("wh_front_right_inv") ||
        !nh_private.hasParam("wh_back_left_diam")
            || !nh_private.hasParam("wh_back_left_idx") ||
            !nh_private.hasParam("wh_back_left_inv") ||
        !nh_private.hasParam("wh_back_right_diam") ||
            !nh_private.hasParam("wh_back_right_idx") ||
            !nh_private.hasParam("wh_back_right_inv")) {
      throw std::runtime_error(
          "[OdomWhROS.cpp] OdomWhROS::readParam: "
          "the steering geometry " + steering_geometry_ + " requires the "
          "definition of the following parameters: "
          "rob_dist_between_front_back_wh, rob_dist_between_left_right_wh, "
          "wh_front_left_diam, wh_front_left_idx, wh_front_left_inv, "
          "wh_front_right_diam, wh_front_right_idx, wh_front_right_inv, "
          "wh_back_left_diam, wh_back_left_idx, wh_back_left_inv, "
          "wh_back_right_diam, wh_back_right_idx, wh_back_right_inv");
    }

    // - robot lengths
    std::vector<double> rob_len(2);
    nh_private.getParam("rob_dist_between_front_back_wh",
                        rob_len[OdomWhOmni4::kRobLenIdxF2B]);
    nh_private.getParam("rob_dist_between_left_right_wh",
                        rob_len[OdomWhOmni4::kRobLenIdxL2R]);

    // - wheel diameters
    std::vector<double> wh_d(4);
    nh_private.getParam("wh_front_left_diam" , wh_d[OdomWhOmni4::kWhIdxFL]);
    nh_private.getParam("wh_front_right_diam", wh_d[OdomWhOmni4::kWhIdxFR]);
    nh_private.getParam("wh_back_left_diam"  , wh_d[OdomWhOmni4::kWhIdxBL]);
    nh_private.getParam("wh_back_right_diam" , wh_d[OdomWhOmni4::kWhIdxBR]);

    // - wheel indexes (getParam não tem overload para size_t....)
    int wh_idx_tmp;
    std::vector<size_t> wh_idx(4);
    nh_private.getParam("wh_front_left_idx" , wh_idx_tmp);
    wh_idx[OdomWhOmni4::kWhIdxFL] = wh_idx_tmp;
    nh_private.getParam("wh_front_right_idx", wh_idx_tmp);
    wh_idx[OdomWhOmni4::kWhIdxFR] = wh_idx_tmp;
    nh_private.getParam("wh_back_left_idx"  , wh_idx_tmp);
    wh_idx[OdomWhOmni4::kWhIdxBL] = wh_idx_tmp;
    nh_private.getParam("wh_back_right_idx" , wh_idx_tmp);
    wh_idx[OdomWhOmni4::kWhIdxBR] = wh_idx_tmp;

    // - wheel invert direction
    bool wh_inv_tmp;
    std::vector<bool> wh_inv(4);
    nh_private.getParam("wh_front_left_inv" , wh_inv_tmp);
    wh_inv[OdomWhOmni4::kWhIdxFL] = wh_inv_tmp;
    nh_private.getParam("wh_front_right_inv", wh_inv_tmp);
    wh_inv[OdomWhOmni4::kWhIdxFR] = wh_inv_tmp;
    nh_private.getParam("wh_back_left_inv"  , wh_inv_tmp);
    wh_inv[OdomWhOmni4::kWhIdxBL] = wh_inv_tmp;
    nh_private.getParam("wh_back_right_inv" , wh_inv_tmp);
    wh_inv[OdomWhOmni4::kWhIdxBR] = wh_inv_tmp;

    // - create odom object
    odom_.reset(new OdomWhOmni4(wh_idx, wh_d, wh_inv, rob_len));

    // - print parameters
    ROS_INFO("[sdpo_ros_odom]   Distance front-back: %lf m",
             odom_->rob_l[OdomWhOmni4::kRobLenIdxF2B]);
    ROS_INFO("[sdpo_ros_odom]   Distance left-right: %lf m",
             odom_->rob_l[OdomWhOmni4::kRobLenIdxL2R]);
    ROS_INFO("[sdpo_ros_odom]   Motor indexes (0 1 2 3): "
             "[%s %s %s %s] (FL|FR|BL|BR)",
             odom_->getMotorDriveIdxStr(0).c_str(),
             odom_->getMotorDriveIdxStr(1).c_str(),
             odom_->getMotorDriveIdxStr(2).c_str(),
             odom_->getMotorDriveIdxStr(3).c_str());
    ROS_INFO("[sdpo_ros_odom]   Wheel diameters (FL FR BL BR): "
             "[%lf %lf %lf %lf] m",
             odom_->mot[OdomWhOmni4::kWhIdxFL].wh_d,
             odom_->mot[OdomWhOmni4::kWhIdxFR].wh_d,
             odom_->mot[OdomWhOmni4::kWhIdxBL].wh_d,
             odom_->mot[OdomWhOmni4::kWhIdxBR].wh_d);
    ROS_INFO("[sdpo_ros_odom]   Wheel inverted (FL FR BL BR): "
             "[%d %d %d %d] (0|1)",
             odom_->mot[OdomWhOmni4::kWhIdxFL].inverted,
             odom_->mot[OdomWhOmni4::kWhIdxFR].inverted,
             odom_->mot[OdomWhOmni4::kWhIdxBL].inverted,
             odom_->mot[OdomWhOmni4::kWhIdxBR].inverted);

  } else {
    throw std::runtime_error(
        "[OdomWhROS.cpp] OdomWhROS::readParam: "
        "invalid steering_geometry (check documentation for supported ones)");
  }

  return true;
};

void OdomWhROS::subMotEnc(const sdpo_ros_interfaces_hw::mot_enc_array& msg) {
  try {
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
    odom2base_tf.frame_id_ = odom_frame_id_;
    odom2base_tf.child_frame_id_ = base_frame_id_;
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
  }
}

void OdomWhROS::subCmdVel(const geometry_msgs::Twist& msg) {
  odom_->setVelRef(msg.linear.x, msg.linear.y, msg.angular.z);

  sdpo_ros_interfaces_hw::mot_ref mot_ref_msg;
  mot_ref_msg.angular_speed_ref.resize(odom_->mot.size());
  for(int i = 0; i < odom_->mot.size(); i++) {
    mot_ref_msg.angular_speed_ref[i] = odom_->getMotorDriveWr(i);
  }

  pub_mot_ref_.publish(mot_ref_msg);
}

} // namespace sdpo_ros_odom
