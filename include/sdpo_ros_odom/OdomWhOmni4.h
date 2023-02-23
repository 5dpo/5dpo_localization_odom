#pragma once

#include "sdpo_ros_odom/OdomWh.h"

namespace sdpo_ros_odom {

const std::string kOdomWhOmni4Str = "omni4";

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
      const std::vector<double>& wh_d, const std::vector<bool>& wh_inv,
      const std::vector<double>& rob_len);

  inline OdomWhType getOdomWhType() const override {
    return OdomWhType::kOmni4Wh;
  }

  void setMotorDriveEncTicksDelta(const size_t& idx, const int32_t& delta_ticks,
      const double& ticks_rev) override;
  void setMotorDriveW(const size_t& idx, const double& w_curr) override;
  double getMotorDriveWr(const size_t& idx) override;
  std::string getMotorDriveIdxStr(const size_t& idx) override;

  void setVelRef(const double& v, const double& vn, const double& w) override;

  void updateOdomDelta() override;
};

} // namespace sdpo_ros_odom
