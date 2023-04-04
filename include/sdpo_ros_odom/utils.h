#pragma once

#include <iostream>
#include <vector>
#include <cmath>

namespace sdpo_ros_odom {

inline float dist(float x, float y) {
  return sqrtf(x * x + y * y);
}

inline double dist(double x, double y) {
  return sqrt(x * x + y * y);
}

inline float normAngDeg(float angle) {
  // Source: https://stackoverflow.com/a/11498248
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0) {
    angle += 360.0f;
  }
  return angle - 180.0f;
}

inline double normAngDeg(double angle) {
  // Source: https://stackoverflow.com/a/11498248
  angle = fmod(angle + 180.0, 360.0);
  if (angle < 0) {
    angle += 360.0;
  }
  return angle - 180.0;
}

inline float normAngRad(float angle) {
  // Source: https://stackoverflow.com/a/11498248
  angle = fmodf(angle + M_PIf32, M_PIf32 * 2.0f);
  if (angle < 0) {
    angle += M_PIf32 * 2.0f;
  }
  return angle - M_PIf32;
}

inline double normAngRad(double angle) {
  // Source: https://stackoverflow.com/a/11498248
  angle = fmod(angle + M_PIf64, M_PIf64 * 2.0);
  if (angle < 0) {
    angle += M_PIf64 * 2.0;
  }
  return angle - M_PIf64;
}

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
