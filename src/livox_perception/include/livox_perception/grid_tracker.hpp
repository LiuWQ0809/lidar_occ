#ifndef LIVOX_PERCEPTION__GRID_TRACKER_HPP_
#define LIVOX_PERCEPTION__GRID_TRACKER_HPP_

#include <cstdint>
#include <vector>

namespace livox_perception {

struct TrackerConfig {
  double occupancy_increase_rate = 0.7;  // probability assigned when cell is occupied
  double decay_rate = 0.05;              // probability decay per cycle when not observed
  double min_probability = 0.1;
  double max_probability = 0.98;
  double occupied_threshold = 0.65;
};

class GridTracker {
 public:
  explicit GridTracker(const TrackerConfig & config);

  void Reset(std::size_t grid_size);

  std::vector<int8_t> Update(const std::vector<int8_t> & new_grid);

 private:
  TrackerConfig config_;
  std::vector<double> probabilities_;
  bool initialized_ = false;
};

}  // namespace livox_perception

#endif  // LIVOX_PERCEPTION__GRID_TRACKER_HPP_
