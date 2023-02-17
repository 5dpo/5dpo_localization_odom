#pragma once

#include "sdpo_ros_odom/OdomWh.h"

namespace sdpo_ros_odom {

class OdomWhOmni4 : public OdomWh {
 public:
  static const size_t kWhIdxFL = 0;
  static const size_t kWhIdxFR = 1;
  static const size_t kWhIdxBL = 2;
  static const size_t kWhIdxBR = 3;

  static const size_t kRobLenIdxF2B = 0;
  static const size_t kRobLenIdxL2R = 1;

 public:
  OdomWhOmni4() = delete;
  OdomWhOmni4(const std::vector<size_t>& wh_idx,
      const std::vector<double>& wh_d, const std::vector<double>& rob_len);

  OdomWhType getOdomWhType() override;

  void setMotorDriveEncTicksDelta(const size_t& idx, const int32_t& delta_ticks,
      const double& ticks_rev) override;
  void setMotorDriveW(const size_t& idx, const double& w_curr) override;

  void setVelRef(const double& v, const double& vn, const double& w) override;

  void update() override;
};

} // namespace sdpo_ros_odom
