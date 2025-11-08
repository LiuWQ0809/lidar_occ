#ifndef LIVOX_PERCEPTION__GRID_MAPPER_HPP_
#define LIVOX_PERCEPTION__GRID_MAPPER_HPP_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"

namespace livox_perception {

struct GridParameters {
  double resolution = 0.1;         // meters per cell
  double width_m = 30.0;           // total width in meters (X axis)
  double height_m = 6.0;           // total height in meters (Y axis)
  double origin_x = -15.0;         // map origin relative to lidar (X axis)
  double origin_y = -3.0;          // map origin relative to lidar (Y axis)
  double origin_z = -2.0;          // used for completeness
  double occupancy_fill_value = 100.0;  // occupancy update value
};

class CudaOccupancyIntegrator;

class GridMapper {
 public:
  GridMapper(const GridParameters & params, std::shared_ptr<CudaOccupancyIntegrator> integrator);

  // 修改：支持坐标系变换
  nav_msgs::msg::OccupancyGrid CreateOccupancyGridMsg(
      const rclcpp::Time & stamp, 
      const std::string & frame_id,
      const Eigen::Matrix4f & transform = Eigen::Matrix4f::Identity()) const;

  std::vector<int8_t> BuildOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & filtered_points);

  const GridParameters & params() const { return params_; }

 private:
  GridParameters params_;
  std::shared_ptr<CudaOccupancyIntegrator> integrator_;
  mutable nav_msgs::msg::OccupancyGrid template_msg_;
};

}  // namespace livox_perception

#endif  // LIVOX_PERCEPTION__GRID_MAPPER_HPP_
