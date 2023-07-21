#include "sdpo_localization_odom/utils.h"

#include <algorithm>
#include <exception>

namespace sdpo_localization_odom {

std::vector<size_t> idx2valueVector(const std::vector<size_t>& vec_ini) {
  auto min_value_it = std::min_element(vec_ini.begin(), vec_ini.end());
  auto max_value_it = std::max_element(vec_ini.begin(), vec_ini.end());
  // Error verification
  // - vector has a valid minimum and maximum element
  if ((vec_ini.size() <= 1) || ((*min_value_it) == (*max_value_it))) {
    throw std::invalid_argument(
        "[utils.cpp] idx2valueVector: invalid vector to convert index 2 range "
        "and vice-versa");
  }
  // - minimum must be 0 + maximum value must be size vector - 1
  if ((*min_value_it != 0) || (*max_value_it != vec_ini.size()-1)) {
    std::cout << *min_value_it << " " << *max_value_it << std::endl;
    throw std::invalid_argument(
        "[utils.cpp] idx2valueVector: invalid vector (range: 0..n-1; "
        "unique values: 0..n-1)");
  }

  std::vector<size_t> vec_new(vec_ini.size());
  for(size_t i = 0; i < vec_ini.size(); i++) {
    vec_new[i] = std::find(vec_ini.begin(), vec_ini.end(), i) - vec_ini.begin();
  }

  return std::move(vec_new);
}

} // namespace sdpo_localization_odom
