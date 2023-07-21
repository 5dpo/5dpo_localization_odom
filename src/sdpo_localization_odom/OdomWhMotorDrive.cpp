#include "sdpo_localization_odom/OdomWhMotorDrive.h"

#include <cmath>

namespace sdpo_localization_odom {

void OdomWhMotorDrive::setEncTicksDelta(const int32_t& delta_ticks) {
  enc_ticks_delta = delta_ticks;

  ang_delta = inverted? -enc_ticks_delta * 2 * M_PIf64 / ticks_per_rev :
                        enc_ticks_delta * 2 * M_PIf64 / ticks_per_rev;

  dist_delta = ang2lin(ang_delta);
}

void OdomWhMotorDrive::setEncTicksDelta(const int32_t& delta_ticks,
                                        const double& ticks_rev) {
  ticks_per_rev = ticks_rev;
  setEncTicksDelta(delta_ticks);
}

void OdomWhMotorDrive::setDistDelta(const double& delta_dist) {
  dist_delta = delta_dist;

  ang_delta = lin2ang(dist_delta);

  enc_ticks_delta = inverted?
      -std::round(ang_delta * ticks_per_rev / (2 * M_PIf64)) :
      std::round(ang_delta * ticks_per_rev / (2 * M_PIf64));
}

void OdomWhMotorDrive::setDistDelta(const double& delta_dist,
                                    const double& ticks_rev) {
  ticks_per_rev = ticks_rev;
  setDistDelta(delta_dist);
}

void OdomWhMotorDrive::setW(const double& w_curr) {
  w = w_curr;
  v = inverted? -ang2lin(w) : ang2lin(w);
}

void OdomWhMotorDrive::setWr(const double& w_ref) {
  w_r = w_ref;
  v_r = inverted? -ang2lin(w_r) : ang2lin(w_r);
}

void OdomWhMotorDrive::setV(const double& v_curr) {
  v = v_curr;
  w = inverted? -lin2ang(v) : lin2ang(v);
}

void OdomWhMotorDrive::setVr(const double& v_ref) {
  v_r = v_ref;
  w_r = inverted? -lin2ang(v_r) : lin2ang(v_r);
}

} // namespace sdpo_localization_odom
