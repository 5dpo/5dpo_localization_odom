#include "sdpo_ros_odom/OdomWhMotorDrive.h"

#include <cmath>

namespace sdpo_ros_odom {

void OdomWhMotorDrive::setEncTicksDelta(const int32_t& delta_ticks,
                                        const double& ticks_rev) {
  enc_ticks_delta = delta_ticks;
  ticks_per_rev = ticks_rev;

  ang_delta = inverted? -enc_ticks_delta * 2 * M_PI / ticks_per_rev :
                        enc_ticks_delta * 2 * M_PI / ticks_per_rev;

  dist_delta = ang2lin(ang_delta);
}

void OdomWhMotorDrive::setW(const double& w_curr) {
  w = w_curr;
  v = inverted? -ang2lin(w) : ang2lin(w);
}

void OdomWhMotorDrive::setVr(const double& v_ref) {
  v_r = v_ref;
  w_r = inverted? -lin2ang(v_r) : lin2ang(v_r);
}

} // namespace sdpo_ros_odom
