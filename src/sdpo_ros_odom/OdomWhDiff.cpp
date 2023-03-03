#include "sdpo_ros_odom/OdomWhDiff.h"

#include <exception>
#include <cmath>

namespace sdpo_ros_odom {

OdomWhDiff::OdomWhDiff(const std::vector<size_t>& wh_idx,
    const std::vector<double>& wh_d, const std::vector<bool>& wh_inv,
    const std::vector<double>& rob_len) {
  if ((wh_idx.size() != 2) || (wh_d.size() != 2) || (wh_inv.size() != 2) ||
      (rob_len.size() != 1)) {
    throw std::invalid_argument(
        "[OdomWhDiff.cpp] OdomWhDiff::OdomWhDiff: "
        "number expected of arguments is 2 wheel motors (index, diameter, "
        "inverted) and 1 distances (left-to-right wheel)");
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
        std::string("[OdomWhDiff.cpp] OdomWhDiff::OdomWhDiff: "
                    "invalid wh_idx vector (") + e.what() + ")");
  }

  // Wheel diameters
  mot[kWhIdxR].wh_d = wh_d[kWhIdxR];
  mot[kWhIdxL].wh_d = wh_d[kWhIdxL];

  // Wheel invert direction
  mot[kWhIdxR].inverted = wh_inv[kWhIdxR];
  mot[kWhIdxL].inverted = wh_inv[kWhIdxL];

  // Robot odometry geometric distances
  rob_l[kRobLenIdx] = rob_len[kRobLenIdx];
}

std::string OdomWhDiff::getMotorDriveIdxStr(const size_t& idx) {
  switch (mot_idx[idx]) {
    case kWhIdxR: return "R";
    case kWhIdxL: return "L";
    default: return "nan";
  }
}

void OdomWhDiff::setVelRef(const double &v, const double &vn,
    const double &w) {
  mot[kWhIdxR].setVr(-v - 0.5 * rob_l[kRobLenIdx] * w);
  mot[kWhIdxL].setVr( v - 0.5 * rob_l[kRobLenIdx] * w);
}

void OdomWhDiff::updateOdomVel() {
  vel.v = 0.5 * (-mot[kWhIdxR].v + mot[kWhIdxL].v);
  vel.vn = 0;
  vel.w = -(mot[kWhIdxR].v + mot[kWhIdxL].v) / rob_l[kRobLenIdx];
}

void OdomWhDiff::updateOdomDelta() {
  // Robot coordinate frame
  odo.u_delta = 0.5 * (-mot[kWhIdxR].dist_delta + mot[kWhIdxL].dist_delta);
  odo.v_delta = 0;
  odo.w_delta = -(mot[kWhIdxR].dist_delta + mot[kWhIdxL].dist_delta) /
      rob_l[kRobLenIdx];

  // Odom global frame
  odo.x_delta = odo.u_delta * cos(pose.th+odo.w_delta/2);
  odo.y_delta = odo.u_delta * sin(pose.th+odo.w_delta/2);
  odo.th_delta = odo.w_delta;
}

} // namespace sdpo_ros_odom
