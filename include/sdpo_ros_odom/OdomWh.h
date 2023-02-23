#pragma once

#include <iostream>
#include <vector>

#include "sdpo_ros_odom/utils.h"
#include "sdpo_ros_odom/OdomWhMotorDrive.h"

namespace sdpo_ros_odom {

enum class OdomWhType {
  kUnknown,
  kDiff,
  kTricyc,
  kOmni3Wh,
  kOmni4Wh
};

class OdomWh {
 public:
  OdomPose2D pose;
  OdomVel2D vel;
  OdomDelta2D odo;

  std::vector<OdomWhMotorDrive> mot;
  std::vector<size_t> mot_idx;
  bool w_r_max_enabled = false;
  double w_r_max = 0;

  std::vector<double> rob_l;

 public:
  OdomWh() = default;
  OdomWh(const std::vector<size_t>& wh_idx, const std::vector<double>& wh_d,
      const std::vector<bool>& wh_inv, const std::vector<double>& rob_len) { }

  inline virtual OdomWhType getOdomWhType() const {
    return OdomWhType::kUnknown;
  }

  virtual void setMotorDriveEncTicksDelta(const size_t& idx,
      const int32_t& delta_ticks, const double& ticks_rev) = 0;
  virtual void setMotorDriveW(const size_t& idx, const double& w_curr) = 0;
  virtual double getMotorDriveWr(const size_t& idx) = 0;
  virtual std::string getMotorDriveIdxStr(const size_t& idx) = 0;

  virtual void setVelRef(const double& v, const double& vn,
      const double& w) = 0;

  void update() {
    updateOdomDelta();
    updateOdomPose();
  }

 private:
  virtual void updateOdomDelta() = 0;
  void updateOdomPose() {
    pose.x += odo.x_delta;
    pose.y += odo.y_delta;
    pose.th += odo.th_delta;
  }
};

} // namespace sdpo_ros_odom
