#include "sdpo_localization_odom/OdomWhROS1.h"

#include <exception>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>

#include "sdpo_localization_odom/OdomWhDiff.h"
#include "sdpo_localization_odom/OdomWhOmni3.h"
#include "sdpo_localization_odom/OdomWhOmni4.h"

namespace sdpo_localization_odom {



OdomWhROS1::OdomWhROS1() : nh_priv_("~")
{

  try
  {
    readParam();
  }
  catch (std::exception& e)
  {
    ROS_FATAL("Error reading the node parameters (%s)", e.what());

    ros::shutdown();

    return;
  }



  pub_cmd_vel_ref_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_ref", 10);

  pub_mot_ref_ = nh_.advertise
      <sdpo_drivers_interfaces::MotRefArrayROS1>("motors_ref", 10);

  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);



  sub_mot_enc_ = nh_.subscribe
      <sdpo_drivers_interfaces::MotEncArrayROS1>(
          "motors_enc", 10, &OdomWhROS1::subMotEnc, this);

  sub_cmd_vel_ = nh_.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 10, &OdomWhROS1::subCmdVel, this);

}



void OdomWhROS1::readParam() {

  if (!nh_priv_.hasParam("steering_geometry"))
  {
    throw std::runtime_error(
        "[OdomWhROS1.cpp] OdomWhROS1::readParam: "
        "the node sdpo_localization_odom requires the definition of the "
        "steering_geometry parameter");
  }



  nh_priv_.param<std::string>
      ("base_frame_id", base_frame_id_, "base_footprint");

  nh_priv_.param<std::string>
      ("odom_frame_id", odom_frame_id_, "odom");

  nh_priv_.param("publish_tf", publish_tf_, true);

  nh_priv_.param("invert_tf", invert_tf_, false);



  nh_priv_.param("w_ref_max_enabled", w_ref_max_enabled_, false);



  if (w_ref_max_enabled_)
  {
    if (!nh_priv_.hasParam("w_ref_max"))
    {
      throw std::runtime_error(
          "[OdomWhROS1.cpp] OdomWhROS1::readParam: "
          "if the maximum angular speed is enabled, the parameter w_ref_max "
          "must be set");
    }

    nh_priv_.getParam("w_ref_max", w_ref_max_);
  }



  nh_priv_.getParam("steering_geometry", steering_geometry_);



  if (steering_geometry_ == kOdomWhOmni4Str)
  {

    if (!nh_priv_.hasParam("rob_dist_between_front_back_wh") ||
        !nh_priv_.hasParam("rob_dist_between_left_right_wh") ||
        !nh_priv_.hasParam("wh_front_left_diam") ||
        !nh_priv_.hasParam("wh_front_left_idx") ||
        !nh_priv_.hasParam("wh_front_left_inv") ||
        !nh_priv_.hasParam("wh_front_right_diam") ||
        !nh_priv_.hasParam("wh_front_right_idx") ||
        !nh_priv_.hasParam("wh_front_right_inv") ||
        !nh_priv_.hasParam("wh_back_left_diam") ||
        !nh_priv_.hasParam("wh_back_left_idx") ||
        !nh_priv_.hasParam("wh_back_left_inv") ||
        !nh_priv_.hasParam("wh_back_right_diam") ||
        !nh_priv_.hasParam("wh_back_right_idx") ||
        !nh_priv_.hasParam("wh_back_right_inv"))
    {
      throw std::runtime_error(
          "[OdomWhROS1.cpp] OdomWhROS1::readParam: "
          "the steering geometry " + steering_geometry_ + " requires the "
          "definition of the following parameters: "
          "rob_dist_between_front_back_wh, rob_dist_between_left_right_wh, "
          "wh_front_left_diam, wh_front_left_idx, wh_front_left_inv, "
          "wh_front_right_diam, wh_front_right_idx, wh_front_right_inv, "
          "wh_back_left_diam, wh_back_left_idx, wh_back_left_inv, "
          "wh_back_right_diam, wh_back_right_idx, wh_back_right_inv");
    }



    std::vector<double> rob_len(2);

    nh_priv_.getParam("rob_dist_between_front_back_wh",
        rob_len[OdomWhOmni4::kRobLenIdxF2B]);

    nh_priv_.getParam("rob_dist_between_left_right_wh",
        rob_len[OdomWhOmni4::kRobLenIdxL2R]);



    std::vector<double> wh_d(4);

    nh_priv_.getParam("wh_front_left_diam", wh_d[OdomWhOmni4::kWhIdxFL]);
    nh_priv_.getParam("wh_front_right_diam", wh_d[OdomWhOmni4::kWhIdxFR]);
    nh_priv_.getParam("wh_back_left_diam", wh_d[OdomWhOmni4::kWhIdxBL]);
    nh_priv_.getParam("wh_back_right_diam", wh_d[OdomWhOmni4::kWhIdxBR]);



    int wh_idx_tmp;
    std::vector<size_t> wh_idx(4);

    nh_priv_.getParam("wh_front_left_idx", wh_idx_tmp);
    wh_idx[OdomWhOmni4::kWhIdxFL] = static_cast<size_t>(wh_idx_tmp);

    nh_priv_.getParam("wh_front_right_idx", wh_idx_tmp);
    wh_idx[OdomWhOmni4::kWhIdxFR] = static_cast<size_t>(wh_idx_tmp);

    nh_priv_.getParam("wh_back_left_idx", wh_idx_tmp);
    wh_idx[OdomWhOmni4::kWhIdxBL] = static_cast<size_t>(wh_idx_tmp);

    nh_priv_.getParam("wh_back_right_idx", wh_idx_tmp);
    wh_idx[OdomWhOmni4::kWhIdxBR] = static_cast<size_t>(wh_idx_tmp);



    bool wh_inv_tmp;
    std::vector<bool> wh_inv(4);

    nh_priv_.getParam("wh_front_left_inv", wh_inv_tmp);
    wh_inv[OdomWhOmni4::kWhIdxFL] = wh_inv_tmp;

    nh_priv_.getParam("wh_front_right_inv", wh_inv_tmp);
    wh_inv[OdomWhOmni4::kWhIdxFR] = wh_inv_tmp;

    nh_priv_.getParam("wh_back_left_inv", wh_inv_tmp);
    wh_inv[OdomWhOmni4::kWhIdxBL] = wh_inv_tmp;

    nh_priv_.getParam("wh_back_right_inv", wh_inv_tmp);
    wh_inv[OdomWhOmni4::kWhIdxBR] = wh_inv_tmp;



    odom_.reset(new OdomWhOmni4(wh_idx, wh_d, wh_inv, rob_len));

  }
  else if (steering_geometry_ == kOdomWhOmni3Str)
  {

    if (!nh_priv_.hasParam("rob_dist_center_wh") ||
        !nh_priv_.hasParam("wh_front_right_diam") ||
        !nh_priv_.hasParam("wh_front_right_idx") ||
        !nh_priv_.hasParam("wh_front_right_inv") ||
        !nh_priv_.hasParam("wh_front_left_diam") ||
        !nh_priv_.hasParam("wh_front_left_idx") ||
        !nh_priv_.hasParam("wh_front_left_inv") ||
        !nh_priv_.hasParam("wh_back_diam") ||
        !nh_priv_.hasParam("wh_back_idx") ||
        !nh_priv_.hasParam("wh_back_inv"))
    {
      throw std::runtime_error(
          "[OdomWhROS1.cpp] OdomWhROS1::readParam: "
          "the steering geometry " + steering_geometry_ + " requires the "
          "definition of the following parameters: "
          "rob_dist_center_wh, "
          "wh_front_right_diam, wh_front_right_idx, wh_front_right_inv, "
          "wh_front_left_diam, wh_front_left_idx, wh_front_left_inv, "
          "wh_back_diam, wh_back_idx, wh_back_inv");
    }



    std::vector<double> rob_len(1);

    nh_priv_.getParam("rob_dist_center_wh", rob_len[OdomWhOmni3::kRobLenIdx]);



    std::vector<double> wh_d(3);

    nh_priv_.getParam("wh_front_right_diam", wh_d[OdomWhOmni3::kWhIdxFR]);
    nh_priv_.getParam("wh_front_left_diam", wh_d[OdomWhOmni3::kWhIdxFL]);
    nh_priv_.getParam("wh_back_diam", wh_d[OdomWhOmni3::kWhIdxB]);




    int wh_idx_tmp;
    std::vector<size_t> wh_idx(3);

    nh_priv_.getParam("wh_front_right_idx", wh_idx_tmp);
    wh_idx[OdomWhOmni3::kWhIdxFR] = wh_idx_tmp;

    nh_priv_.getParam("wh_front_left_idx", wh_idx_tmp);
    wh_idx[OdomWhOmni3::kWhIdxFL] = wh_idx_tmp;

    nh_priv_.getParam("wh_back_idx", wh_idx_tmp);
    wh_idx[OdomWhOmni3::kWhIdxB] = wh_idx_tmp;



    bool wh_inv_tmp;
    std::vector<bool> wh_inv(3);

    nh_priv_.getParam("wh_front_right_inv", wh_inv_tmp);
    wh_inv[OdomWhOmni3::kWhIdxFR] = wh_inv_tmp;

    nh_priv_.getParam("wh_front_left_inv", wh_inv_tmp);
    wh_inv[OdomWhOmni3::kWhIdxFL] = wh_inv_tmp;

    nh_priv_.getParam("wh_back_inv", wh_inv_tmp);
    wh_inv[OdomWhOmni3::kWhIdxB] = wh_inv_tmp;



    odom_.reset(new OdomWhOmni3(wh_idx, wh_d, wh_inv, rob_len));

  }
  else if (steering_geometry_ == kOdomWhDiffStr)
  {

    if (!nh_priv_.hasParam("rob_dist_between_wh") ||
        !nh_priv_.hasParam("wh_right_diam") ||
        !nh_priv_.hasParam("wh_right_idx") ||
        !nh_priv_.hasParam("wh_right_inv") ||
        !nh_priv_.hasParam("wh_left_diam") ||
        !nh_priv_.hasParam("wh_left_idx") ||
        !nh_priv_.hasParam("wh_left_inv"))
    {
      throw std::runtime_error(
          "[OdomWhROS1.cpp] OdomWhROS1::readParam: "
          "the steering geometry " + steering_geometry_ + " requires the "
          "definition of the following parameters: "
          "rob_dist_between_wh, "
          "wh_right_diam, wh_right_idx, wh_right_inv, "
          "wh_left_diam, wh_left_idx, wh_left_inv");
    }



    std::vector<double> rob_len(1);

    nh_priv_.getParam("rob_dist_between_wh", rob_len[OdomWhDiff::kRobLenIdx]);



    std::vector<double> wh_d(2);

    nh_priv_.getParam("wh_right_diam", wh_d[OdomWhDiff::kWhIdxR]);
    nh_priv_.getParam("wh_left_diam", wh_d[OdomWhDiff::kWhIdxL]);



    int wh_idx_tmp;
    std::vector<size_t> wh_idx(2);

    nh_priv_.getParam("wh_right_idx", wh_idx_tmp);
    wh_idx[OdomWhDiff::kWhIdxR] = wh_idx_tmp;

    nh_priv_.getParam("wh_left_idx", wh_idx_tmp);
    wh_idx[OdomWhDiff::kWhIdxL] = wh_idx_tmp;



    bool wh_inv_tmp;
    std::vector<bool> wh_inv(2);

    nh_priv_.getParam("wh_right_inv", wh_inv_tmp);
    wh_inv[OdomWhDiff::kWhIdxR] = wh_inv_tmp;

    nh_priv_.getParam("wh_left_inv", wh_inv_tmp);
    wh_inv[OdomWhDiff::kWhIdxL] = wh_inv_tmp;



    odom_.reset(new OdomWhDiff(wh_idx, wh_d, wh_inv, rob_len));

  }
  /// @todo TRYCLE STEERING GEOMETRY
  /*else if (steering_geometry_ == tricycle)
  {
    this->declare_parameter("tricyc_rob_dist_between_wh");
    this->declare_parameter("tricyc_wh_front_drive_diam");
    this->declare_parameter("tricyc_wh_front_drive_idx");
    this->declare_parameter("tricyc_wh_front_drive_inv");
    this->declare_parameter("tricyc_wh_front_steer_offs");
    this->declare_parameter("tricyc_wh_front_steer_idx");
    this->declare_parameter("tricyc_wh_front_steer_inv");
  }*/
  else
  {
    throw std::runtime_error(
        "[OdomWhROS1.cpp] OdomWhROS1::readParam: "
        "invalid steering_geometry (check documentation for supported ones)");
  }



  if (w_ref_max_enabled_)
  {
    odom_->setMotorWRefMax(w_ref_max_enabled_, w_ref_max_);
  }





  ROS_INFO("[%s] Base frame ID: %s",
           ros::this_node::getName().c_str(), base_frame_id_.c_str());

  ROS_INFO("[%s] Odom frame ID: %s",
           ros::this_node::getName().c_str(), odom_frame_id_.c_str());

  ROS_INFO("[%s] Publish TF: %s",
           ros::this_node::getName().c_str(), publish_tf_? "yes" : "no");

  ROS_INFO("[%s] Invert TF: %s",
           ros::this_node::getName().c_str(), invert_tf_? "yes" : "no");

  ROS_INFO("[%s] Maximum wheel angular speed enabled: %s",
           ros::this_node::getName().c_str(), w_ref_max_enabled_? "yes" : "no");

  if (w_ref_max_enabled_)
  {
    ROS_INFO("[%s] Maximum wheel angular speed: %lf (rad/s)",
             ros::this_node::getName().c_str(), w_ref_max_);
  }



  ROS_INFO("[%s] Steering geometry: %s",
           ros::this_node::getName().c_str(), steering_geometry_.c_str());



  if (steering_geometry_ == kOdomWhOmni4Str)
  {
    ROS_INFO("[%s] Distance front-back: %lf m",
             ros::this_node::getName().c_str(),
             odom_->rob_l[OdomWhOmni4::kRobLenIdxF2B]);

    ROS_INFO("[%s] Distance left-right: %lf m",
             ros::this_node::getName().c_str(),
             odom_->rob_l[OdomWhOmni4::kRobLenIdxL2R]);

    ROS_INFO("[%s] Motor indexes (0 1 2 3): "
             "[%s %s %s %s] (FL|FR|BL|BR)",
             ros::this_node::getName().c_str(),
             odom_->getMotorDriveIdxStr(0).c_str(),
             odom_->getMotorDriveIdxStr(1).c_str(),
             odom_->getMotorDriveIdxStr(2).c_str(),
             odom_->getMotorDriveIdxStr(3).c_str());

    ROS_INFO("[%s] Wheel diameters (FL FR BL BR): "
             "[%lf %lf %lf %lf] m",
             ros::this_node::getName().c_str(),
             odom_->mot[OdomWhOmni4::kWhIdxFL].wh_d,
             odom_->mot[OdomWhOmni4::kWhIdxFR].wh_d,
             odom_->mot[OdomWhOmni4::kWhIdxBL].wh_d,
             odom_->mot[OdomWhOmni4::kWhIdxBR].wh_d);

    ROS_INFO("[%s] Wheel inverted (FL FR BL BR): "
             "[%d %d %d %d] (0|1)",
             ros::this_node::getName().c_str(),
             odom_->mot[OdomWhOmni4::kWhIdxFL].inverted,
             odom_->mot[OdomWhOmni4::kWhIdxFR].inverted,
             odom_->mot[OdomWhOmni4::kWhIdxBL].inverted,
             odom_->mot[OdomWhOmni4::kWhIdxBR].inverted);
  }
  else if (steering_geometry_ == kOdomWhOmni3Str)
  {
    ROS_INFO("[%s] Distance center-wheels: %lf m",
             ros::this_node::getName().c_str(),
             odom_->rob_l[OdomWhOmni3::kRobLenIdx]);

    ROS_INFO("[%s] Motor indexes (0 1 2): "
             "[%s %s %s] (FR|FL|B)",
             ros::this_node::getName().c_str(),
             odom_->getMotorDriveIdxStr(0).c_str(),
             odom_->getMotorDriveIdxStr(1).c_str(),
             odom_->getMotorDriveIdxStr(2).c_str());

    ROS_INFO("[%s] Wheel diameters (FR FL B): "
             "[%lf %lf %lf] m",
             ros::this_node::getName().c_str(),
             odom_->mot[OdomWhOmni3::kWhIdxFR].wh_d,
             odom_->mot[OdomWhOmni3::kWhIdxFL].wh_d,
             odom_->mot[OdomWhOmni3::kWhIdxB].wh_d);

    ROS_INFO("[%s] Wheel inverted (FR FL B): "
             "[%d %d %d] (0|1)",
             ros::this_node::getName().c_str(),
             odom_->mot[OdomWhOmni3::kWhIdxFR].inverted,
             odom_->mot[OdomWhOmni3::kWhIdxFL].inverted,
             odom_->mot[OdomWhOmni3::kWhIdxB].inverted);
  }
  else if (steering_geometry_ == kOdomWhDiffStr)
  {
    ROS_INFO("[%s] Distance between wheels: %lf m",
             ros::this_node::getName().c_str(),
             odom_->rob_l[OdomWhDiff::kRobLenIdx]);
    ROS_INFO("[%s] Motor indexes (0 1): "
             "[%s %s] (R|L)",
             ros::this_node::getName().c_str(),
             odom_->getMotorDriveIdxStr(0).c_str(),
             odom_->getMotorDriveIdxStr(1).c_str());
    ROS_INFO("[%s] Wheel diameters (R L): "
             "[%lf %lf] m",
             ros::this_node::getName().c_str(),
             odom_->mot[OdomWhDiff::kWhIdxR].wh_d,
             odom_->mot[OdomWhDiff::kWhIdxL].wh_d);
    ROS_INFO("[%s] Wheel inverted (R L): "
             "[%d %d] (0|1)",
             ros::this_node::getName().c_str(),
             odom_->mot[OdomWhDiff::kWhIdxR].inverted,
             odom_->mot[OdomWhDiff::kWhIdxL].inverted);
  }
}



void OdomWhROS1::subMotEnc(
    const sdpo_drivers_interfaces::MotEncArrayROS1::ConstPtr& msg)
{

  try
  {
    for(size_t i = 0; i < msg->mot_enc.size(); i++)
    {
      odom_->setMotorDriveEncTicksDelta(
          i, msg->mot_enc[i].enc_delta, msg->mot_enc[i].ticks_per_rev);

      odom_->setMotorDriveW(i, msg->mot_enc[i].ang_speed);
    }

    odom_->update();



    geometry_msgs::Twist odom_vel;

    odom_vel.linear.x = odom_->vel.v;
    odom_vel.linear.y = odom_->vel.vn;
    odom_vel.linear.z = 0.0;
    odom_vel.angular.x = 0.0;
    odom_vel.angular.y = 0.0;
    odom_vel.angular.z = odom_->vel.w;



    geometry_msgs::TransformStamped odom2base_tf;
    tf2::Quaternion odom_rot_q;

    if (invert_tf_)
    {
      odom2base_tf.header.stamp = msg->stamp;
      odom2base_tf.header.frame_id = base_frame_id_;
      odom2base_tf.child_frame_id  = odom_frame_id_;

      odom2base_tf.transform.translation.x = -odom_->pose.x;
      odom2base_tf.transform.translation.y = -odom_->pose.y;
      odom2base_tf.transform.translation.z = 0.0;

      odom_rot_q.setRPY(0.0, 0.0, -odom_->pose.th);
    }
    else
    {
      odom2base_tf.header.stamp = msg->stamp;
      odom2base_tf.header.frame_id = odom_frame_id_;
      odom2base_tf.child_frame_id  = base_frame_id_;

      odom2base_tf.transform.translation.x = odom_->pose.x;
      odom2base_tf.transform.translation.y = odom_->pose.y;
      odom2base_tf.transform.translation.z = 0.0;

      odom_rot_q.setRPY(0.0, 0.0, odom_->pose.th);
    }

    odom2base_tf.transform.rotation.x = odom_rot_q.x();
    odom2base_tf.transform.rotation.y = odom_rot_q.y();
    odom2base_tf.transform.rotation.z = odom_rot_q.z();
    odom2base_tf.transform.rotation.w = odom_rot_q.w();

    if (publish_tf_)
    {
      tf_broad_.sendTransform(odom2base_tf);
    }



    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = odom2base_tf.header.stamp;
    odom_msg.header.frame_id = odom2base_tf.header.frame_id;
    odom_msg.child_frame_id = odom2base_tf.child_frame_id;
    odom_msg.pose.pose.position.x = odom2base_tf.transform.translation.x;
    odom_msg.pose.pose.position.y = odom2base_tf.transform.translation.y;
    odom_msg.pose.pose.position.z = odom2base_tf.transform.translation.z;
    odom_msg.pose.pose.orientation.x = odom2base_tf.transform.rotation.x;
    odom_msg.pose.pose.orientation.y = odom2base_tf.transform.rotation.y;
    odom_msg.pose.pose.orientation.z = odom2base_tf.transform.rotation.z;
    odom_msg.pose.pose.orientation.w = odom2base_tf.transform.rotation.w;
    odom_msg.twist.twist = odom_vel;

    pub_odom_.publish(odom_msg);

  }
  catch (std::exception& e)
  {
    ROS_ERROR("[%s] Error when processing the motor encoders data message (%s)",
              ros::this_node::getName().c_str(), e.what());
  }

}



void OdomWhROS1::subCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{

  odom_->setVelRef(msg->linear.x, msg->linear.y, msg->angular.z);



  sdpo_drivers_interfaces::MotRefArrayROS1 mot_ref_msg;

  mot_ref_msg.stamp = ros::Time::now();
  mot_ref_msg.ang_speed_ref.resize(odom_->mot.size());

  for(size_t i = 0; i < odom_->mot.size(); i++) {
    mot_ref_msg.ang_speed_ref[i].ref = odom_->getMotorDriveWr(i);
  }

  pub_mot_ref_.publish(mot_ref_msg);



  if (odom_->w_r_max_enabled) {
    pubCmdVelRef();
  }

}



void OdomWhROS1::pubCmdVelRef() {

  geometry_msgs::Twist cmd_vel_ref;

  cmd_vel_ref.linear.x = odom_->vel.v_r;
  cmd_vel_ref.linear.y = odom_->vel.vn_r;
  cmd_vel_ref.linear.z = 0;

  cmd_vel_ref.angular.x = 0;
  cmd_vel_ref.angular.y = 0;
  cmd_vel_ref.angular.z = odom_->vel.w_r;



  pub_cmd_vel_ref_.publish(cmd_vel_ref);

}

} // namespace sdpo_localization_odom
