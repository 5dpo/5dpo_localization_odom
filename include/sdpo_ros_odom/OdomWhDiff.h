#pragma once

#include "sdpo_ros_odom/OdomWh.h"

namespace sdpo_ros_odom {

const std::string kOdomWhDiffStr = "diff";

class OdomWhDiff : public OdomWh {
 public:
  static const size_t kWhIdxR = 0;
  static const size_t kWhIdxL = 1;

  static const size_t kRobLenIdx = 0;

 public:
  OdomWhDiff() = delete;
  OdomWhDiff(const std::vector<size_t>& wh_idx,
      const std::vector<double>& wh_d, const std::vector<bool>& wh_inv,
      const std::vector<double>& rob_len);

  inline OdomWhType getOdomWhType() const override {
    return OdomWhType::kDiff;
  }

  std::string getMotorDriveIdxStr(const size_t& idx) override;

  void setVelRef(const double& v, const double& vn, const double& w) override;
  void getVelRef(double& v, double& vn, double& w) override;

  void updateOdomVel() override;
  void updateOdomDelta() override;
};

} // namespace sdpo_ros_odom
