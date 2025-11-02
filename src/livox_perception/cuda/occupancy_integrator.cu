#include "livox_perception/cuda_occupancy_integrator.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

namespace livox_perception {
namespace {

constexpr int kThreadsPerBlock = 256;

inline void CheckCuda(cudaError_t status, const char * expr) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string("CUDA error: ") + cudaGetErrorString(status) +
                             " when calling " + expr);
  }
}

__global__ void ResetKernel(int32_t * grid, int cell_count, int32_t value) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= cell_count) {
    return;
  }
  grid[idx] = value;
}

__global__ void OccupancyKernel(const float * points, int point_count, int width, int height,
                                float resolution, float origin_x, float origin_y,
                                int32_t hit_value, int32_t * grid) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= point_count) {
    return;
  }

  const int offset = idx * 3;
  const float x = points[offset];
  const float y = points[offset + 1];

  const int gx = static_cast<int>(floorf((x - origin_x) / resolution));
  const int gy = static_cast<int>(floorf((y - origin_y) / resolution));

  if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
    return;
  }

  const int cell_index = gy * width + gx;
  atomicMax(&grid[cell_index], hit_value);
}

}  // namespace

CudaOccupancyIntegrator::CudaOccupancyIntegrator() = default;

CudaOccupancyIntegrator::~CudaOccupancyIntegrator() {
  if (device_points_ != nullptr) {
    cudaFree(device_points_);
    device_points_ = nullptr;
  }
  if (device_grid_ != nullptr) {
    cudaFree(device_grid_);
    device_grid_ = nullptr;
  }
}

void CudaOccupancyIntegrator::AllocatePointBuffer(std::size_t count) {
  if (count <= device_points_capacity_) {
    return;
  }
  if (device_points_ != nullptr) {
    cudaFree(device_points_);
    device_points_ = nullptr;
  }
  const std::size_t bytes = count * sizeof(float) * 3U;
  CheckCuda(cudaMalloc(&device_points_, bytes), "cudaMalloc(device_points_)");
  device_points_capacity_ = count;
}

void CudaOccupancyIntegrator::AllocateGridBuffer(std::size_t cell_count) {
  if (cell_count <= device_grid_capacity_) {
    return;
  }
  if (device_grid_ != nullptr) {
    cudaFree(device_grid_);
    device_grid_ = nullptr;
  }
  const std::size_t bytes = cell_count * sizeof(int32_t);
  CheckCuda(cudaMalloc(&device_grid_, bytes), "cudaMalloc(device_grid_)");
  device_grid_capacity_ = cell_count;
}

void CudaOccupancyIntegrator::Integrate(const std::vector<Eigen::Vector3f> & points,
                                        const CudaGridSpec & spec,
                                        std::vector<int8_t> * grid) {
  if (grid == nullptr) {
    throw std::invalid_argument("Grid pointer must not be null");
  }
  const std::size_t cell_count = static_cast<std::size_t>(spec.width * spec.height);
  if (grid->size() != cell_count) {
    grid->assign(cell_count, -1);
  }

  if (points.empty()) {
    std::fill(grid->begin(), grid->end(), -1);
    return;
  }

  AllocatePointBuffer(points.size());
  AllocateGridBuffer(cell_count);

  std::vector<float> host_points(points.size() * 3U);
  for (std::size_t i = 0; i < points.size(); ++i) {
    host_points[i * 3U] = points[i].x();
    host_points[i * 3U + 1U] = points[i].y();
    host_points[i * 3U + 2U] = points[i].z();
  }

  CheckCuda(cudaMemcpy(device_points_, host_points.data(), host_points.size() * sizeof(float),
                       cudaMemcpyHostToDevice), "cudaMemcpy(points)");

  const int threads = kThreadsPerBlock;
  const int point_blocks = static_cast<int>((points.size() + threads - 1) / threads);
  const int grid_blocks = static_cast<int>((cell_count + threads - 1) / threads);

  ResetKernel<<<grid_blocks, threads>>>(device_grid_, static_cast<int>(cell_count), -1);
  CheckCuda(cudaGetLastError(), "ResetKernel launch");

  OccupancyKernel<<<point_blocks, threads>>>(device_points_, static_cast<int>(points.size()),
                                            spec.width, spec.height, spec.resolution,
                                            spec.origin_x, spec.origin_y,
                                            static_cast<int32_t>(spec.hit_value), device_grid_);
  CheckCuda(cudaGetLastError(), "OccupancyKernel launch");
  CheckCuda(cudaDeviceSynchronize(), "cudaDeviceSynchronize");

  std::vector<int32_t> host_grid(cell_count);
  CheckCuda(cudaMemcpy(host_grid.data(), device_grid_, cell_count * sizeof(int32_t),
                       cudaMemcpyDeviceToHost), "cudaMemcpy(grid)");

  for (std::size_t idx = 0; idx < cell_count; ++idx) {
    if (host_grid[idx] < 0) {
      (*grid)[idx] = -1;
      continue;
    }

    if (host_grid[idx] >= static_cast<int32_t>(spec.hit_value)) {
      const float clamped_hit = std::max(0.0F, std::min(spec.hit_value, 100.0F));
      (*grid)[idx] = static_cast<int8_t>(clamped_hit);
      continue;
    }

    (*grid)[idx] = 0;
  }
}

}  // namespace livox_perception
