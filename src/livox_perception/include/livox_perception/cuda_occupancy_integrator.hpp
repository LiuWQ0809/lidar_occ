#ifndef LIVOX_PERCEPTION__CUDA_OCCUPANCY_INTEGRATOR_HPP_
#define LIVOX_PERCEPTION__CUDA_OCCUPANCY_INTEGRATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "Eigen/Core"

namespace livox_perception {

struct CudaGridSpec {
  int width = 0;
  int height = 0;
  float resolution = 0.1F;
  float origin_x = 0.0F;
  float origin_y = 0.0F;
  float hit_value = 100.0F;
  float free_value = 0.0F;
};

class CudaOccupancyIntegrator {
 public:
  CudaOccupancyIntegrator();
  ~CudaOccupancyIntegrator();

  CudaOccupancyIntegrator(const CudaOccupancyIntegrator &) = delete;
  CudaOccupancyIntegrator & operator=(const CudaOccupancyIntegrator &) = delete;

  void Integrate(const std::vector<Eigen::Vector3f> & points,
                 const CudaGridSpec & spec,
                 std::vector<int8_t> * grid);

 private:
  void AllocatePointBuffer(std::size_t count);
  void AllocateGridBuffer(std::size_t cell_count);

  float * device_points_ = nullptr;
  int32_t * device_grid_ = nullptr;
  std::size_t device_points_capacity_ = 0U;
  std::size_t device_grid_capacity_ = 0U;
};

}  // namespace livox_perception

#endif  // LIVOX_PERCEPTION__CUDA_OCCUPANCY_INTEGRATOR_HPP_
