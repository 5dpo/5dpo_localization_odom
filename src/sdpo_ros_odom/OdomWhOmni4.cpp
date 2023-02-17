#include "sdpo_ros_odom/OdomWhOmni4.h"

#include <exception>

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

OdomWhType OdomWhOmni4::getOdomWhType() {
  return OdomWhType::kOmni4Wh;
}

void OdomWhOmni4::setMotorDriveEncTicksDelta(const size_t &idx,
    const int32_t &delta_ticks, const double &ticks_rev) {
  mot[mot_idx[idx]].setEncTicksDelta(delta_ticks, ticks_rev);
}

void OdomWhOmni4::setMotorDriveW(const size_t &idx, const double &w_curr) {
  mot[mot_idx[idx]].setW(w_curr);
}

void OdomWhOmni4::setVelRef(const double &v, const double &vn,
    const double &w) {

}

void OdomWhOmni4::update() {

}

} // namespace sdpo_ros_odom
