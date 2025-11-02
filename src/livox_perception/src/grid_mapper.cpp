#include "livox_perception/grid_mapper.hpp"

#include <algorithm>
#include <cmath>

#include "Eigen/Geometry"
#include "livox_perception/cuda_occupancy_integrator.hpp"

namespace livox_perception {
namespace {
constexpr double kDefaultYaw = 0.0;
}  // namespace

GridMapper::GridMapper(const GridParameters & params,
                       std::shared_ptr<CudaOccupancyIntegrator> integrator)
    : params_(params), integrator_(std::move(integrator)) {
  const auto width_cells = static_cast<uint32_t>(std::ceil(params_.width_m / params_.resolution));
  const auto height_cells = static_cast<uint32_t>(std::ceil(params_.height_m / params_.resolution));

  template_msg_.info.resolution = static_cast<float>(params_.resolution);
  template_msg_.info.width = width_cells;
  template_msg_.info.height = height_cells;
  template_msg_.info.origin.position.x = params_.origin_x;
  template_msg_.info.origin.position.y = params_.origin_y;
  template_msg_.info.origin.position.z = params_.origin_z;
  template_msg_.info.origin.orientation.x = 0.0;
  template_msg_.info.origin.orientation.y = 0.0;
  template_msg_.info.origin.orientation.z = std::sin(kDefaultYaw * 0.5);
  template_msg_.info.origin.orientation.w = std::cos(kDefaultYaw * 0.5);
  template_msg_.data.assign(width_cells * height_cells, -1);
}

nav_msgs::msg::OccupancyGrid GridMapper::CreateOccupancyGridMsg(
    const rclcpp::Time & stamp,
    const std::string & frame_id,
    const Eigen::Matrix4f & transform) const {
  auto msg = template_msg_;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  
  // 修改：如果提供了变换矩阵，将网格的origin变换到目标坐标系
  if (!transform.isApprox(Eigen::Matrix4f::Identity())) {
    // 将lidar坐标系下的网格原点变换到camera坐标系
    Eigen::Vector4f origin_lidar(
        static_cast<float>(params_.origin_x),
        static_cast<float>(params_.origin_y),
        static_cast<float>(params_.origin_z),
        1.0f);
    
    Eigen::Vector4f origin_camera = transform * origin_lidar;
    
    msg.info.origin.position.x = static_cast<double>(origin_camera.x());
    msg.info.origin.position.y = static_cast<double>(origin_camera.y());
    msg.info.origin.position.z = static_cast<double>(origin_camera.z());
    
    // 从变换矩阵中提取旋转并转换为四元数
    Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation);
    quat.normalize();
    
    msg.info.origin.orientation.x = static_cast<double>(quat.x());
    msg.info.origin.orientation.y = static_cast<double>(quat.y());
    msg.info.origin.orientation.z = static_cast<double>(quat.z());
    msg.info.origin.orientation.w = static_cast<double>(quat.w());
  }
  
  return msg;
}

std::vector<int8_t> GridMapper::BuildOccupancyGrid(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & filtered_points) {
  const auto width_cells = static_cast<int32_t>(template_msg_.info.width);
  const auto height_cells = static_cast<int32_t>(template_msg_.info.height);
  std::vector<int8_t> grid(width_cells * height_cells, -1);

  if (filtered_points == nullptr || filtered_points->empty()) {
    return grid;
  }

  std::vector<Eigen::Vector3f> host_points;
  host_points.reserve(filtered_points->size());
  for (const auto & point : filtered_points->points) {
    host_points.emplace_back(point.x, point.y, point.z);
  }

  CudaGridSpec spec;
  spec.width = width_cells;
  spec.height = height_cells;
  spec.resolution = static_cast<float>(params_.resolution);
  spec.origin_x = static_cast<float>(params_.origin_x);
  spec.origin_y = static_cast<float>(params_.origin_y);
  spec.hit_value = static_cast<float>(params_.occupancy_fill_value);
  spec.free_value = 0.0F;

  integrator_->Integrate(host_points, spec, &grid);
  return grid;
}

}  // namespace livox_perception
