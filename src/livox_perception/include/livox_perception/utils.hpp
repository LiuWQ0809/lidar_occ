#ifndef LIVOX_PERCEPTION__UTILS_HPP_
#define LIVOX_PERCEPTION__UTILS_HPP_

#include <cstddef>
#include <vector>

#include "Eigen/Core"

namespace livox_perception {

inline std::size_t GridIndex(int x, int y, int width) {
  return static_cast<std::size_t>(y * width + x);
}

inline Eigen::Vector2f GridToWorld(int x, int y, float resolution, float origin_x, float origin_y) {
  return Eigen::Vector2f(origin_x + (static_cast<float>(x) + 0.5F) * resolution,
                         origin_y + (static_cast<float>(y) + 0.5F) * resolution);
}

}  // namespace livox_perception

#endif  // LIVOX_PERCEPTION__UTILS_HPP_
