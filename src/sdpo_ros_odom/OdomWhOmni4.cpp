#include "sdpo_ros_odom/OdomWhOmni4.h"

#include <exception>
#include <cmath>

namespace sdpo_ros_odom {

OdomWhOmni4::OdomWhOmni4(const std::vector<size_t>& wh_idx,
    const std::vector<double>& wh_d, const std::vector<double>& rob_len) {
  if ((wh_idx.size() != 4) || (wh_d.size() != 4) || (rob_len.size() != 2)) {
    throw std::invalid_argument(
        "[OdomWhOmni4.cpp] OdomWhOmni4::OdomWhOmni4: "
        "number expected of arguments is 4 wheel motors (diameter and index) "
        "and 2 distances (front-to-back and left-to-right wheels)");
  }

  // Initialization
  mot_idx.resize(wh_idx.size());
  mot.resize(wh_d.size());
  rob_l.resize(rob_len.size());

  // Motor indexes
  try {
    mot_idx = idx2valueVector(wh_idx);
  } catch (std::exception& e) {
    throw std::invalid_argument(
        std::string("[OdomWhOmni4.cpp] OdomWhOmni4::OdomWhOmni4: "
        "invalid wh_idx vector (") + e.what() + ")");
  }

  // Wheel diameters
  mot[kWhIdxFL].wh_d = wh_d[kWhIdxFL];
  mot[kWhIdxFR].wh_d = wh_d[kWhIdxFR];
  mot[kWhIdxBL].wh_d = wh_d[kWhIdxBL];
  mot[kWhIdxBR].wh_d = wh_d[kWhIdxBR];

  // Robot odometry geometric distances
  rob_l[kRobLenIdxF2B] = rob_len[kRobLenIdxF2B];
  rob_l[kRobLenIdxL2R] = rob_len[kRobLenIdxL2R];
}

void OdomWhOmni4::setMotorDriveEncTicksDelta(const size_t &idx,
    const int32_t &delta_ticks, const double &ticks_rev) {
  mot[mot_idx[idx]].setEncTicksDelta(delta_ticks, ticks_rev);
}

void OdomWhOmni4::setMotorDriveW(const size_t &idx, const double &w_curr) {
  mot[mot_idx[idx]].setW(w_curr);
}

double OdomWhOmni4::getMotorDriveWr(const size_t& idx) {
  return mot[mot_idx[idx]].w_r;
}

void OdomWhOmni4::setVelRef(const double &v, const double &vn,
    const double &w) {
  mot[kWhIdxFL].setVr( v - vn - (rob_l[kRobLenIdxF2B]+rob_l[kRobLenIdxL2R])/2);
  mot[kWhIdxFR].setVr(-v - vn - (rob_l[kRobLenIdxF2B]+rob_l[kRobLenIdxL2R])/2);
  mot[kWhIdxBL].setVr( v + vn - (rob_l[kRobLenIdxF2B]+rob_l[kRobLenIdxL2R])/2);
  mot[kWhIdxBR].setVr(-v + vn - (rob_l[kRobLenIdxF2B]+rob_l[kRobLenIdxL2R])/2);
}

void OdomWhOmni4::updateOdomDelta() {
  // Robot coordinate frame
  odo.u_delta = (
      mot[kWhIdxFL].dist_delta - mot[kWhIdxFR].dist_delta +
      mot[kWhIdxBL].dist_delta - mot[kWhIdxBR].dist_delta) / 4;
  odo.v_delta =(
      -mot[kWhIdxFL].dist_delta - mot[kWhIdxFR].dist_delta +
      mot[kWhIdxBL].dist_delta + mot[kWhIdxBR].dist_delta) / 4;
  odo.w_delta = -(
      mot[kWhIdxFL].dist_delta + mot[kWhIdxFR].dist_delta +
      mot[kWhIdxBL].dist_delta + mot[kWhIdxBR].dist_delta) /
          (2 * (rob_l[kRobLenIdxF2B] + rob_l[kRobLenIdxL2R]));

  // Odom global frame
  if (odo.w_delta == 0) {
    odo.x_delta = odo.u_delta * cos(pose.th) - odo.v_delta * sin(pose.th);
    odo.y_delta = odo.u_delta * sin(pose.th) + odo.v_delta * cos(pose.th);
  } else {
    odo.x_delta =
        (odo.u_delta*sin(odo.w_delta) + odo.v_delta * (cos(odo.w_delta)-1)) *
            cos(pose.th+odo.w_delta/2) / odo.w_delta -
        (odo.u_delta*(1-cos(odo.w_delta)) + odo.v_delta * sin(odo.w_delta)) *
            sin(pose.th+odo.w_delta/2) / odo.w_delta;
    odo.y_delta =
        (odo.u_delta*sin(odo.w_delta) + odo.v_delta * (cos(odo.w_delta)-1)) *
            sin(pose.th+odo.w_delta/2) / odo.w_delta +
        (odo.u_delta*(1-cos(odo.w_delta)) + odo.v_delta * sin(odo.w_delta)) *
            cos(pose.th+odo.w_delta/2) / odo.w_delta;
  }
  odo.th_delta = odo.w_delta;
}

} // namespace sdpo_ros_odom
