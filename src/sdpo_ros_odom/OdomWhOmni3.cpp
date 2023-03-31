#include "sdpo_ros_odom/OdomWhOmni3.h"

#include <exception>
#include <cmath>

namespace sdpo_ros_odom {

OdomWhOmni3::OdomWhOmni3(const std::vector<size_t>& wh_idx,
    const std::vector<double>& wh_d, const std::vector<bool>& wh_inv,
    const std::vector<double>& rob_len) {
  if ((wh_idx.size() != 3) || (wh_d.size() != 3) || (wh_inv.size() != 3) ||
      (rob_len.size() != 1)) {
    throw std::invalid_argument(
        "[OdomWhOmni3.cpp] OdomWhOmni3::OdomWhOmni3: "
        "number expected of arguments is 3 wheel motors (index, diameter, "
        "inverted) and 1 distances (geometric center-to-wheels)");
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
        std::string("[OdomWhOmni3.cpp] OdomWhOmni3::OdomWhOmni3: "
                    "invalid wh_idx vector (") + e.what() + ")");
  }

  // Wheel diameters
  mot[kWhIdxFR].wh_d = wh_d[kWhIdxFR];
  mot[kWhIdxFL].wh_d = wh_d[kWhIdxFL];
  mot[kWhIdxB].wh_d = wh_d[kWhIdxB];

  // Wheel invert direction
  mot[kWhIdxFR].inverted = wh_inv[kWhIdxFR];
  mot[kWhIdxFL].inverted = wh_inv[kWhIdxFL];
  mot[kWhIdxB].inverted = wh_inv[kWhIdxB];

  // Robot odometry geometric distances
  rob_l[kRobLenIdx] = rob_len[kRobLenIdx];
}

std::string OdomWhOmni3::getMotorDriveIdxStr(const size_t& idx) {
  switch (mot_idx[idx]) {
    case kWhIdxFR: return "FR";
    case kWhIdxFL: return "FL";
    case kWhIdxB: return "B";
    default: return "nan";
  }
}

void OdomWhOmni3::setVelRef(const double &v, const double &vn,
    const double &w) {
  mot[kWhIdxFR].setVr(
      -0.5 * sqrt(3) * v - 0.5 * vn - rob_l[kRobLenIdx] * w);
  mot[kWhIdxFL].setVr(
       0.5 * sqrt(3) * v - 0.5 * vn - rob_l[kRobLenIdx] * w);
  mot[kWhIdxB].setVr(vn - rob_l[kRobLenIdx] * w);
}

void OdomWhOmni3::getVelRef(double& v, double& vn, double& w) {
  v  = sqrt(3) * (-mot[kWhIdxFR].v_r + mot[kWhIdxFL].v_r) / 3;
  vn = (-mot[kWhIdxFR].v_r - mot[kWhIdxFL].v_r + 2 * mot[kWhIdxB].v_r) / 3;
  w  = -(mot[kWhIdxFR].v_r + mot[kWhIdxFL].v_r + mot[kWhIdxB].v_r) /
      (3 * rob_l[kRobLenIdx]);
}

void OdomWhOmni3::updateOdomVel() {
  vel.v = sqrt(3) * (-mot[kWhIdxFR].v + mot[kWhIdxFL].v) / 3;
  vel.vn = (-mot[kWhIdxFR].v - mot[kWhIdxFL].v + 2 * mot[kWhIdxB].v) / 3;
  vel.w = -(mot[kWhIdxFR].v + mot[kWhIdxFL].v + mot[kWhIdxB].v) /
      (3 * rob_l[kRobLenIdx]);
}

void OdomWhOmni3::updateOdomDelta() {
  // Robot coordinate frame
  odo.u_delta = sqrt(3) * (
      -mot[kWhIdxFR].dist_delta + mot[kWhIdxFL].dist_delta) / 3;
  odo.v_delta = (
      -mot[kWhIdxFR].dist_delta - mot[kWhIdxFL].dist_delta +
      2 * mot[kWhIdxB].dist_delta) / 3;
  odo.w_delta = -(
      mot[kWhIdxFR].dist_delta + mot[kWhIdxFL].dist_delta +
      mot[kWhIdxB].dist_delta) / (3 * rob_l[kRobLenIdx]);

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
