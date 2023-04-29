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

  inline void setMotorWRefMax(const bool& enable, const double& w_ref_max) {
    w_r_max_enabled = enable;
    if (w_r_max_enabled) {
      w_r_max = w_ref_max;
    }
  }

  inline void setMotorDriveTicksRev(const double& ticks_rev) {
    for (auto& motor : mot) {
      motor.ticks_per_rev = ticks_rev;
    }
  }
  inline void setMotorDriveEncTicksDelta(const size_t& idx,
      const int32_t& delta_ticks, const double& ticks_rev) {
    mot[mot_idx[idx]].setEncTicksDelta(delta_ticks, ticks_rev);
  }
  inline void setMotorDriveDistDelta(const size_t& idx,
      const double& delta_dist, const double& ticks_rev) {
    mot[mot_idx[idx]].setDistDelta(delta_dist, ticks_rev);
  }
  inline void setMotorDriveW(const size_t& idx, const double& w_curr) {
    mot[mot_idx[idx]].setW(w_curr);
  }
  inline void setMotorDriveWr(const size_t& idx, const double& w_ref) {
    mot[mot_idx[idx]].setWr(w_ref);
  }
  inline void setMotorDriveV(const size_t& idx, const double& v_curr) {
    mot[mot_idx[idx]].setV(v_curr);
  }
  inline void setMotorDriveVr(const size_t& idx, const double& v_ref) {
    mot[mot_idx[idx]].setVr(v_ref);
  }

  inline size_t getMotorDriveIdx(const size_t& idx) {
    return mot_idx[idx];
  }
  virtual std::string getMotorDriveIdxStr(const size_t& idx) = 0;

  inline void getMotorDriveEncTicksDelta(const size_t& idx,
      int32_t& delta_ticks, double& ticks_rev) {
    delta_ticks = mot[mot_idx[idx]].enc_ticks_delta;
    ticks_rev = mot[mot_idx[idx]].ticks_per_rev;
  }
  inline double getMotorDriveW(const size_t& idx) {
    return mot[mot_idx[idx]].w;
  }
  inline double getMotorDriveWr(const size_t& idx) {
    return mot[mot_idx[idx]].w_r;
  }

  void setVel(const double& v, const double& vn, const double& w) {
    vel.v  = v;
    vel.vn = vn;
    vel.w  = w;

    updateVelInv();
  }
  void setVelRef(const double& v_ref, const double& vn_ref, const double& w_ref){
    vel.v_r = v_ref;
    vel.vn_r = vn_ref;
    vel.w_r = w_ref;

    updateVelRefInv();

    if (w_r_max_enabled) {
      scaleMotorsDriveWr();
    }
  }
  virtual void updateVelRef() = 0;

  void update() {
    updateVel();
    updatePose();
  }

  void updateInv(const double& x, const double& y, const double& th) {
    updateVelInv();
    updatePoseInv(x, y, th);
  }

  virtual void computeFwdKin(const std::vector<double>& v_mot,
      double& v, double& vn, double& w) = 0;
  virtual void computeInvKin(const double& v, const double& vn, const double& w,
      std::vector<double>& v_mot) = 0;

 protected:
  virtual void updateVel() = 0;
  virtual void updateVelInv() = 0;
  virtual void updateVelRefInv() = 0;

  void updatePose() {
    updateOdomDelta();

    pose.x += odo.x_delta;
    pose.y += odo.y_delta;
    pose.th += odo.th_delta;
  }
  void updatePoseInv(const double& x, const double& y, const double& th) {
    odo.x_delta  = x - pose.x;
    odo.y_delta  = y - pose.y;
    odo.th_delta = normAngRad(th - pose.th);

    updateOdomDeltaInv();

    pose.x = x;
    pose.y = y;
    pose.th = th;
  }

  virtual void updateOdomDelta() = 0;
  virtual void updateOdomDeltaInv() = 0;

  inline void scaleMotorsDriveWr() {
    double curr_w_r_max = 0;
    
    // Get absolute maximum wheel angular velocity
    for (auto& m : mot) {
      if (std::abs(m.w_r) > curr_w_r_max) {
        curr_w_r_max = std::abs(m.w_r);
      }
    }

    // Estimate scale if needed + estimate new angular velocities and robot ref
    if ((curr_w_r_max > w_r_max) && (w_r_max != 0)) {
      double scale = w_r_max / curr_w_r_max;

      for (auto& m : mot) {
        m.setWr(m.w_r * scale);
      }
      
      updateVelRef();
    }
  }
};

} // namespace sdpo_ros_odom
