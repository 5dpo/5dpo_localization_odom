#pragma once

#include "sdpo_ros_odom/OdomWh.h"

namespace sdpo_ros_odom {

const std::string kOdomWhOmni3Str = "omni3";

class OdomWhOmni3 : public OdomWh {
 public:
  static const size_t kWhIdxFR = 0;
  static const size_t kWhIdxFL = 1;
  static const size_t kWhIdxB = 2;

  static const size_t kRobLenIdx = 0;

 public:
  OdomWhOmni3() = delete;
  OdomWhOmni3(const std::vector<size_t>& wh_idx,
      const std::vector<double>& wh_d, const std::vector<bool>& wh_inv,
      const std::vector<double>& rob_len);

  inline OdomWhType getOdomWhType() const override {
    return OdomWhType::kOmni3Wh;
  }

  std::string getMotorDriveIdxStr(const size_t& idx) override;

  void setVelRef(const double& v, const double& vn, const double& w) override;

  void updateOdomDelta() override;
};

} // namespace sdpo_ros_odom
