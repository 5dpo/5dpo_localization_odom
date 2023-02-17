#include "sdpo_ros_odom/OdomWh.h"

namespace sdpo_ros_odom {

OdomWh::OdomWh(const std::vector<size_t>& wh_idx, const std::vector<double>& wh_d,
       const std::vector<double>& rob_len) { }

OdomWhType OdomWh::getOdomWhType() {
  return OdomWhType::kUnknown;
}

} // namespace sdpo_ros_odom
