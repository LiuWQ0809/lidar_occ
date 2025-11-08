#include "livox_perception/grid_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace livox_perception {

GridTracker::GridTracker(const TrackerConfig & config) : config_(config) {}

void GridTracker::Reset(std::size_t grid_size) {
  probabilities_.assign(grid_size, config_.min_probability);
  initialized_ = true;
}

std::vector<int8_t> GridTracker::Update(const std::vector<int8_t> & new_grid) {
  if (!initialized_ || probabilities_.size() != new_grid.size()) {
    Reset(new_grid.size());
  }

  std::vector<int8_t> tracked(new_grid.size(), -1);

  for (std::size_t idx = 0; idx < new_grid.size(); ++idx) {
    double probability = probabilities_[idx];
    const int8_t measurement = new_grid[idx];

    if (measurement < 0) {
      probability = std::max(config_.min_probability, probability - config_.decay_rate);
    } else {
      const bool occupied = measurement >= 90;
      if (occupied) {
        const double delta = config_.occupancy_increase_rate * (1.0 - probability);
        probability = std::min(config_.max_probability, probability + delta);
      } else {
        const double delta = config_.occupancy_increase_rate * probability;
        probability = std::max(config_.min_probability, probability - delta);
      }
    }

    probabilities_[idx] = probability;

    if (probability >= config_.occupied_threshold) {
      tracked[idx] = 100;
    } else if (measurement >= 0) {
      tracked[idx] = 0;
    } else if (probability <= config_.min_probability + 1e-3) {
      tracked[idx] = -1;
    }
  }

  return tracked;
}

}  // namespace livox_perception
