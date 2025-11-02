#ifndef LIVOX_PERCEPTION__PREPROCESSOR_HPP_
#define LIVOX_PERCEPTION__PREPROCESSOR_HPP_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace livox_perception {

struct PreprocessorConfig {
  double min_distance = 0.1;  // meters
  double max_forward = 15.0;
  double max_backward = 15.0;
  double max_left = 3.0;
  double max_right = 3.0;
  double min_height = -2.0;
  double max_height = 3.0;
  double invalid_rect_front = 0.02;   // meters in +X
  double invalid_rect_back = 0.5;      // meters in -X
  double invalid_rect_right = 0.5;     // meters in -Y direction
  double invalid_rect_left = 0.0;      // meters in +Y from center (right side only)
  double voxel_leaf_size = 0.1;        // meters
};

class Preprocessor {
 public:
  explicit Preprocessor(const PreprocessorConfig & config);

  pcl::PointCloud<pcl::PointXYZ>::Ptr Filter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud) const;

 private:
  bool IsInsideRegion(const Eigen::Vector3f & point) const;
  bool IsInsideInvalidRect(const Eigen::Vector3f & point) const;

  PreprocessorConfig config_;
};

}  // namespace livox_perception

#endif  // LIVOX_PERCEPTION__PREPROCESSOR_HPP_
