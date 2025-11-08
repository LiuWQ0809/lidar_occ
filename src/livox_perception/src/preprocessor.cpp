#include "livox_perception/preprocessor.hpp"

#include <cmath>

#include "pcl/filters/voxel_grid.h"
#include "pcl/common/common.h"

namespace livox_perception {

Preprocessor::Preprocessor(const PreprocessorConfig & config) : config_(config) {}

pcl::PointCloud<pcl::PointXYZ>::Ptr Preprocessor::Filter(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud) const {
  auto cropped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cropped->reserve(cloud->size());

  for (const auto & point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    Eigen::Vector3f p(point.x, point.y, point.z);
    const double distance = p.norm();
    if (distance < config_.min_distance) {
      continue;
    }
    if (!IsInsideRegion(p)) {
      continue;
    }
    if (IsInsideInvalidRect(p)) {
      continue;
    }
    cropped->push_back(point);
  }

  if (cropped->empty()) {
    return cropped;
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cropped);
  voxel_filter.setLeafSize(static_cast<float>(config_.voxel_leaf_size),
                           static_cast<float>(config_.voxel_leaf_size),
                           static_cast<float>(config_.voxel_leaf_size));
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  voxel_filter.filter(*filtered);
  return filtered;
}

bool Preprocessor::IsInsideRegion(const Eigen::Vector3f & point) const {
  const double x = static_cast<double>(point.x());
  const double y = static_cast<double>(point.y());
  const double z = static_cast<double>(point.z());

  if (x > config_.max_forward || x < -config_.max_backward) {
    return false;
  }
  if (y > config_.max_left || y < -config_.max_right) {
    return false;
  }
  if (z < config_.min_height || z > config_.max_height) {
    return false;
  }
  return true;
}

bool Preprocessor::IsInsideInvalidRect(const Eigen::Vector3f & point) const {
  const double x = static_cast<double>(point.x());
  const double y = static_cast<double>(point.y());

  const double front_limit = config_.invalid_rect_front;
  const double back_limit = -config_.invalid_rect_back;
  const double right_limit = -config_.invalid_rect_right;
  const double left_limit = config_.invalid_rect_left;

  if (x <= front_limit && x >= back_limit && y >= right_limit && y <= left_limit) {
    return true;
  }
  return false;
}

}  // namespace livox_perception
