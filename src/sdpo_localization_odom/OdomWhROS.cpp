#include "sdpo_localization_odom/OdomWhROS.h"

#include <exception>
#include <vector>

#include "sdpo_localization_odom/OdomWhDiff.h"
#include "sdpo_localization_odom/OdomWhOmni3.h"
#include "sdpo_localization_odom/OdomWhOmni4.h"

namespace sdpo_localization_odom {



OdomWhROS::OdomWhROS() : rclcpp::Node("sdpo_localization_odom_wh")
{

  try
  {
    readParam();
  }
  catch (std::exception& e)
  {
    RCLCPP_FATAL(this->get_logger(),
                 "Error reading the node parameters (%s)", e.what());

    rclcpp::shutdown();

    return;
  }



  pub_cmd_vel_ref_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel_ref", 10);

  pub_mot_ref_ = this->create_publisher
      <sdpo_drivers_interfaces::msg::MotRefArray>("motors_ref", 10);

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);



  sub_mot_enc_ = this->create_subscription
      <sdpo_drivers_interfaces::msg::MotEncArray>(
          "motors_enc", 10,
          std::bind(&OdomWhROS::subMotEnc, this, std::placeholders::_1));

  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&OdomWhROS::subCmdVel, this, std::placeholders::_1));

}



void OdomWhROS::readParam() {

  this->declare_parameter<std::string>("base_frame_id", "base_footprint");
  this->declare_parameter<std::string>("odom_frame_id", "odom");

  this->declare_parameter<bool>("publish_tf", true);
  this->declare_parameter<bool>("invert_tf", false);

  this->declare_parameter<bool>("w_ref_max_enabled", false);

  this->declare_parameter("w_ref_max");

  this->declare_parameter("steering_geometry");



  if (!this->has_parameter("steering_geometry"))
  {
    throw std::runtime_error(
        "[OdomWhROS.cpp] OdomWhROS::readParam: "
        "the node sdpo_localization_odom requires the definition of the "
        "steering_geometry parameter");
  }



  base_frame_id_ = this->get_parameter("base_frame_id").as_string();

  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();

  publish_tf_ = this->get_parameter("publish_tf").as_bool();

  invert_tf_ = this->get_parameter("invert_tf").as_bool();



  w_ref_max_enabled_ = this->get_parameter("w_ref_max_enabled").as_bool();

  if (w_ref_max_enabled_)
  {
    if (!this->has_parameter("w_ref_max"))
    {
      throw std::runtime_error(
          "[OdomWhROS.cpp] OdomWhROS::readParam: "
          "if the maximum angular speed is enabled, the parameter w_ref_max "
          "must be set");
    }

    w_ref_max_ = this->get_parameter("w_ref_max").as_double();
  }



  steering_geometry_ = this->get_parameter("steering_geometry").as_string();



  if (steering_geometry_ == kOdomWhOmni4Str)
  {
    this->declare_parameter("rob_dist_between_front_back_wh");
    this->declare_parameter("rob_dist_between_left_right_wh");
    this->declare_parameter("wh_front_left_diam");
    this->declare_parameter("wh_front_left_idx");
    this->declare_parameter("wh_front_left_inv");
    this->declare_parameter("wh_front_right_diam");
    this->declare_parameter("wh_front_right_idx");
    this->declare_parameter("wh_front_right_inv");
    this->declare_parameter("wh_back_left_diam");
    this->declare_parameter("wh_back_left_idx");
    this->declare_parameter("wh_back_left_inv");
    this->declare_parameter("wh_back_right_diam");
    this->declare_parameter("wh_back_right_idx");
    this->declare_parameter("wh_back_right_inv");

    if (!this->has_parameter("rob_dist_between_front_back_wh") ||
        !this->has_parameter("rob_dist_between_left_right_wh") ||
        !this->has_parameter("wh_front_left_diam") ||
        !this->has_parameter("wh_front_left_idx") ||
        !this->has_parameter("wh_front_left_inv") ||
        !this->has_parameter("wh_front_right_diam") ||
        !this->has_parameter("wh_front_right_idx") ||
        !this->has_parameter("wh_front_right_inv") ||
        !this->has_parameter("wh_back_left_diam") ||
        !this->has_parameter("wh_back_left_idx") ||
        !this->has_parameter("wh_back_left_inv") ||
        !this->has_parameter("wh_back_right_diam") ||
        !this->has_parameter("wh_back_right_idx") ||
        !this->has_parameter("wh_back_right_inv"))
    {
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



    std::vector<double> rob_len(2);

    rob_len[OdomWhOmni4::kRobLenIdxF2B] =
        this->get_parameter("rob_dist_between_front_back_wh").as_double();
    rob_len[OdomWhOmni4::kRobLenIdxL2R] =
        this->get_parameter("rob_dist_between_left_right_wh").as_double();



    std::vector<double> wh_d(4);

    wh_d[OdomWhOmni4::kWhIdxFL] =
        this->get_parameter("wh_front_left_diam").as_double();
    wh_d[OdomWhOmni4::kWhIdxFR] =
        this->get_parameter("wh_front_right_diam").as_double();
    wh_d[OdomWhOmni4::kWhIdxBL] =
        this->get_parameter("wh_back_left_diam").as_double();
    wh_d[OdomWhOmni4::kWhIdxBR] =
        this->get_parameter("wh_back_right_diam").as_double();



    int wh_idx_tmp;
    std::vector<size_t> wh_idx(4);

    wh_idx_tmp = this->get_parameter("wh_front_left_idx").as_int();
    wh_idx[OdomWhOmni4::kWhIdxFL] = static_cast<size_t>(wh_idx_tmp);

    wh_idx_tmp = this->get_parameter("wh_front_right_idx").as_int();
    wh_idx[OdomWhOmni4::kWhIdxFR] = static_cast<size_t>(wh_idx_tmp);

    wh_idx_tmp = this->get_parameter("wh_back_left_idx").as_int();
    wh_idx[OdomWhOmni4::kWhIdxBL] = static_cast<size_t>(wh_idx_tmp);

    wh_idx_tmp = this->get_parameter("wh_back_right_idx").as_int();
    wh_idx[OdomWhOmni4::kWhIdxBR] = static_cast<size_t>(wh_idx_tmp);



    std::vector<bool> wh_inv(4);

    wh_inv[OdomWhOmni4::kWhIdxFL] =
        this->get_parameter("wh_front_left_inv").as_bool();
    wh_inv[OdomWhOmni4::kWhIdxFR] =
        this->get_parameter("wh_front_right_inv").as_bool();
    wh_inv[OdomWhOmni4::kWhIdxBL] =
        this->get_parameter("wh_back_left_inv").as_bool();
    wh_inv[OdomWhOmni4::kWhIdxBR] =
        this->get_parameter("wh_back_right_inv").as_bool();



    odom_.reset(new OdomWhOmni4(wh_idx, wh_d, wh_inv, rob_len));
  }
  else if (steering_geometry_ == kOdomWhOmni3Str)
  {
    this->declare_parameter("rob_dist_center_wh");
    this->declare_parameter("wh_front_right_diam");
    this->declare_parameter("wh_front_right_idx");
    this->declare_parameter("wh_front_right_inv");
    this->declare_parameter("wh_front_left_diam");
    this->declare_parameter("wh_front_left_idx");
    this->declare_parameter("wh_front_left_inv");
    this->declare_parameter("wh_back_diam");
    this->declare_parameter("wh_back_idx");
    this->declare_parameter("wh_back_inv");

    if (!this->has_parameter("rob_dist_center_wh") ||
        !this->has_parameter("wh_front_right_diam") ||
        !this->has_parameter("wh_front_right_idx") ||
        !this->has_parameter("wh_front_right_inv") ||
        !this->has_parameter("wh_front_left_diam") ||
        !this->has_parameter("wh_front_left_idx") ||
        !this->has_parameter("wh_front_left_inv") ||
        !this->has_parameter("wh_back_diam") ||
        !this->has_parameter("wh_back_idx") ||
        !this->has_parameter("wh_back_inv"))
    {
      throw std::runtime_error(
          "[OdomWhROS.cpp] OdomWhROS::readParam: "
          "the steering geometry " + steering_geometry_ + " requires the "
          "definition of the following parameters: "
          "rob_dist_center_wh, "
          "wh_front_right_diam, wh_front_right_idx, wh_front_right_inv, "
          "wh_front_left_diam, wh_front_left_idx, wh_front_left_inv, "
          "wh_back_diam, wh_back_idx, wh_back_inv");
    }



    std::vector<double> rob_len(1);

    rob_len[OdomWhOmni3::kRobLenIdx] =
        this->get_parameter("rob_dist_center_wh").as_double();



    std::vector<double> wh_d(3);

    wh_d[OdomWhOmni3::kWhIdxFR] =
        this->get_parameter("wh_front_right_diam").as_double();
    wh_d[OdomWhOmni3::kWhIdxFL] =
        this->get_parameter("wh_front_left_diam").as_double();
    wh_d[OdomWhOmni3::kWhIdxB ] =
        this->get_parameter("wh_back_diam").as_double();




    int wh_idx_tmp;
    std::vector<size_t> wh_idx(3);

    wh_idx_tmp = this->get_parameter("wh_front_right_idx").as_int();
    wh_idx[OdomWhOmni3::kWhIdxFR] = static_cast<size_t>(wh_idx_tmp);

    wh_idx_tmp = this->get_parameter("wh_front_left_idx").as_int();
    wh_idx[OdomWhOmni3::kWhIdxFL] = static_cast<size_t>(wh_idx_tmp);

    wh_idx_tmp = this->get_parameter("wh_back_idx").as_int();
    wh_idx[OdomWhOmni3::kWhIdxB] = static_cast<size_t>(wh_idx_tmp);



    std::vector<bool> wh_inv(3);

    wh_inv[OdomWhOmni3::kWhIdxFR] =
        this->get_parameter("wh_front_right_inv").as_bool();
    wh_inv[OdomWhOmni3::kWhIdxFL] =
        this->get_parameter("wh_front_left_inv").as_bool();
    wh_inv[OdomWhOmni3::kWhIdxB ] =
        this->get_parameter("wh_back_inv").as_bool();



    odom_.reset(new OdomWhOmni3(wh_idx, wh_d, wh_inv, rob_len));
  }
  else if (steering_geometry_ == kOdomWhDiffStr)
  {
    this->declare_parameter("rob_dist_between_wh");
    this->declare_parameter("wh_right_diam");
    this->declare_parameter("wh_right_idx");
    this->declare_parameter("wh_right_inv");
    this->declare_parameter("wh_left_diam");
    this->declare_parameter("wh_left_idx");
    this->declare_parameter("wh_left_inv");

    if (!this->has_parameter("rob_dist_between_wh") ||
        !this->has_parameter("wh_right_diam") ||
        !this->has_parameter("wh_right_idx") ||
        !this->has_parameter("wh_right_inv") ||
        !this->has_parameter("wh_left_diam") ||
        !this->has_parameter("wh_left_idx") ||
        !this->has_parameter("wh_left_inv"))
    {
      throw std::runtime_error(
          "[OdomWhROS.cpp] OdomWhROS::readParam: "
          "the steering geometry " + steering_geometry_ + " requires the "
          "definition of the following parameters: "
          "rob_dist_between_wh, "
          "wh_right_diam, wh_right_idx, wh_right_inv, "
          "wh_left_diam, wh_left_idx, wh_left_inv");
    }



    std::vector<double> rob_len(1);

    rob_len[OdomWhDiff::kRobLenIdx] =
        this->get_parameter("rob_dist_between_wh").as_double();



    std::vector<double> wh_d(2);

    wh_d[OdomWhDiff::kWhIdxR] =
        this->get_parameter("wh_right_diam").as_double();
    wh_d[OdomWhDiff::kWhIdxL] =
        this->get_parameter("wh_left_diam").as_double();



    int wh_idx_tmp;
    std::vector<size_t> wh_idx(2);

    wh_idx_tmp = this->get_parameter("wh_right_idx").as_int();
    wh_idx[OdomWhDiff::kWhIdxR] = wh_idx_tmp;

    wh_idx_tmp = this->get_parameter("wh_left_idx").as_int();
    wh_idx[OdomWhDiff::kWhIdxL] = wh_idx_tmp;



    std::vector<bool> wh_inv(2);

    wh_inv[OdomWhDiff::kWhIdxR] = this->get_parameter("wh_right_inv").as_bool();
    wh_inv[OdomWhDiff::kWhIdxL] = this->get_parameter("wh_left_inv").as_bool();



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
        "[OdomWhROS.cpp] OdomWhROS::readParam: "
        "invalid steering_geometry (check documentation for supported ones)");
  }



  if (w_ref_max_enabled_)
  {
    odom_->setMotorWRefMax(w_ref_max_enabled_, w_ref_max_);
  }





  RCLCPP_INFO(this->get_logger(), "Base frame ID: %s", base_frame_id_.c_str());

  RCLCPP_INFO(this->get_logger(), "Odom frame ID: %s", odom_frame_id_.c_str());

  RCLCPP_INFO(this->get_logger(),
              "Publish TF: %s", publish_tf_? "yes" : "no");

  RCLCPP_INFO(this->get_logger(),
              "Invert TF: %s", invert_tf_? "yes" : "no");

  RCLCPP_INFO(this->get_logger(),
              "Maximum wheel angular speed enabled: %s",
              w_ref_max_enabled_? "yes" : "no");

  if (w_ref_max_enabled_)
  {
    RCLCPP_INFO(this->get_logger(),
                "Maximum wheel angular speed: %lf (rad/s)", w_ref_max_);
  }



  RCLCPP_INFO(this->get_logger(),
              "Steering geometry: %s", steering_geometry_.c_str());



  if (steering_geometry_ == kOdomWhOmni4Str)
  {
    RCLCPP_INFO(this->get_logger(),
                "  Distance front-back: %lf m",
                odom_->rob_l[OdomWhOmni4::kRobLenIdxF2B]);

    RCLCPP_INFO(this->get_logger(),
                "  Distance left-right: %lf m",
                odom_->rob_l[OdomWhOmni4::kRobLenIdxL2R]);

    RCLCPP_INFO(this->get_logger(),
                "  Motor indexes (0 1 2 3): "
                "[%s %s %s %s] (FL|FR|BL|BR)",
                odom_->getMotorDriveIdxStr(0).c_str(),
                odom_->getMotorDriveIdxStr(1).c_str(),
                odom_->getMotorDriveIdxStr(2).c_str(),
                odom_->getMotorDriveIdxStr(3).c_str());

    RCLCPP_INFO(this->get_logger(),
                "  Wheel diameters (FL FR BL BR): "
                "[%lf %lf %lf %lf] m",
                odom_->mot[OdomWhOmni4::kWhIdxFL].wh_d,
                odom_->mot[OdomWhOmni4::kWhIdxFR].wh_d,
                odom_->mot[OdomWhOmni4::kWhIdxBL].wh_d,
                odom_->mot[OdomWhOmni4::kWhIdxBR].wh_d);

    RCLCPP_INFO(this->get_logger(),
                "  Wheel inverted (FL FR BL BR): "
                "[%d %d %d %d] (0|1)",
                odom_->mot[OdomWhOmni4::kWhIdxFL].inverted,
                odom_->mot[OdomWhOmni4::kWhIdxFR].inverted,
                odom_->mot[OdomWhOmni4::kWhIdxBL].inverted,
                odom_->mot[OdomWhOmni4::kWhIdxBR].inverted);
  }
  else if (steering_geometry_ == kOdomWhOmni3Str)
  {
    RCLCPP_INFO(this->get_logger(),
                "  Distance center-wheels: %lf m",
                odom_->rob_l[OdomWhOmni3::kRobLenIdx]);

    RCLCPP_INFO(this->get_logger(),
                "  Motor indexes (0 1 2): "
                "[%s %s %s] (FR|FL|B)",
                odom_->getMotorDriveIdxStr(0).c_str(),
                odom_->getMotorDriveIdxStr(1).c_str(),
                odom_->getMotorDriveIdxStr(2).c_str());

    RCLCPP_INFO(this->get_logger(),
                "  Wheel diameters (FR FL B): "
                "[%lf %lf %lf] m",
                odom_->mot[OdomWhOmni3::kWhIdxFR].wh_d,
                odom_->mot[OdomWhOmni3::kWhIdxFL].wh_d,
                odom_->mot[OdomWhOmni3::kWhIdxB].wh_d);

    RCLCPP_INFO(this->get_logger(),
                "  Wheel inverted (FR FL B): "
                "[%d %d %d] (0|1)",
                odom_->mot[OdomWhOmni3::kWhIdxFR].inverted,
                odom_->mot[OdomWhOmni3::kWhIdxFL].inverted,
                odom_->mot[OdomWhOmni3::kWhIdxB].inverted);
  }
  else if (steering_geometry_ == kOdomWhDiffStr)
  {
    RCLCPP_INFO(this->get_logger(),
                "  Distance between wheels: %lf m",
                odom_->rob_l[OdomWhDiff::kRobLenIdx]);
    RCLCPP_INFO(this->get_logger(),
                "  Motor indexes (0 1): "
                "[%s %s] (R|L)",
                odom_->getMotorDriveIdxStr(0).c_str(),
                odom_->getMotorDriveIdxStr(1).c_str());
    RCLCPP_INFO(this->get_logger(),
                "  Wheel diameters (R L): "
                "[%lf %lf] m",
                odom_->mot[OdomWhDiff::kWhIdxR].wh_d,
                odom_->mot[OdomWhDiff::kWhIdxL].wh_d);
    RCLCPP_INFO(this->get_logger(),
                "  Wheel inverted (R L): "
                "[%d %d] (0|1)",
                odom_->mot[OdomWhDiff::kWhIdxR].inverted,
                odom_->mot[OdomWhDiff::kWhIdxL].inverted);
  }
}



void OdomWhROS::subMotEnc(
    const sdpo_drivers_interfaces::msg::MotEncArray::SharedPtr msg)
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



    geometry_msgs::msg::Twist odom_vel;

    odom_vel.linear.x = odom_->vel.v;
    odom_vel.linear.y = odom_->vel.vn;
    odom_vel.linear.z = 0.0;
    odom_vel.angular.x = 0.0;
    odom_vel.angular.y = 0.0;
    odom_vel.angular.z = odom_->vel.w;



    /*tf::StampedTransform odom2base_tf;
    odom2base_tf.setOrigin(tf::Vector3(odom_->pose.x, odom_->pose.y, 0.0));
    odom2base_tf.setRotation(tf::createQuaternionFromYaw(odom_->pose.th));
    odom2base_tf.stamp_ = msg.stamp;
    odom2base_tf.frame_id_ = odom_frame_id_;
    odom2base_tf.child_frame_id_ = base_frame_id_;

    if (publish_tf_) {
      tf_broad_.sendTransform(odom2base_tf);
    }*/



    nav_msgs::msg::Odometry odom_msg;
/*
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
*/
    pub_odom_->publish(odom_msg);

  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Error when processing the motor encoders data message (%s)",
                 e.what());
  }

}



void OdomWhROS::subCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{

  odom_->setVelRef(msg->linear.x, msg->linear.y, msg->angular.z);



  sdpo_drivers_interfaces::msg::MotRefArray mot_ref_msg;

  mot_ref_msg.stamp = this->get_clock()->now();
  mot_ref_msg.ang_speed_ref.resize(odom_->mot.size());

  for(size_t i = 0; i < odom_->mot.size(); i++) {
    mot_ref_msg.ang_speed_ref[i].ref = odom_->getMotorDriveWr(i);
  }

  pub_mot_ref_->publish(mot_ref_msg);



  if (odom_->w_r_max_enabled) {
    pubCmdVelRef();
  }

}



void OdomWhROS::pubCmdVelRef() {

  geometry_msgs::msg::Twist cmd_vel_ref;

  cmd_vel_ref.linear.x = odom_->vel.v_r;
  cmd_vel_ref.linear.y = odom_->vel.vn_r;
  cmd_vel_ref.linear.z = 0;

  cmd_vel_ref.angular.x = 0;
  cmd_vel_ref.angular.y = 0;
  cmd_vel_ref.angular.z = odom_->vel.w_r;



  pub_cmd_vel_ref_->publish(cmd_vel_ref);

}

} // namespace sdpo_localization_odom
