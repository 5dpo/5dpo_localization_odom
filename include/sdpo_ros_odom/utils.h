#pragma once

#include <iostream>
#include <vector>

namespace sdpo_ros_odom {

struct OdomPose2D {
 public:
  double x = 0;
  double y = 0;
  double th = 0;
};

struct OdomVel2D {
 public:
  double v_r = 0;
  double vn_r = 0;
  double w_r = 0;

  double v = 0;
  double vn = 0;
  double w = 0;
};

struct OdomDelta2D {
 public:
  double x_delta = 0;
  double y_delta = 0;
  double th_delta = 0;

  double u_delta = 0;
  double v_delta = 0;
  double w_delta = 0;
};

std::vector<size_t> idx2valueVector(const std::vector<size_t>& vec_ini);

} // namespace sdpo_ros_odom
