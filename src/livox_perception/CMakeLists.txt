cmake_minimum_required(VERSION 3.14)
project(livox_perception LANGUAGES C CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(livox_perception_cuda STATIC
  cuda/occupancy_integrator.cu
)

set_target_properties(livox_perception_cuda PROPERTIES CUDA_ARCHITECTURES 87)

target_include_directories(livox_perception_cuda
  PUBLIC
    include
)

target_link_libraries(livox_perception_cuda
  PUBLIC
    CUDA::cudart
)

target_compile_options(livox_perception_cuda
  PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
)

set(SOURCES
  src/livox_perception_node.cpp
  src/preprocessor.cpp
  src/grid_mapper.cpp
  src/grid_tracker.cpp
  src/utils.cpp
)

add_executable(livox_perception_node ${SOURCES})

target_include_directories(livox_perception_node
  PUBLIC
    include
)

target_link_libraries(livox_perception_node
  livox_perception_cuda
  ${PCL_LIBRARIES}
)

target_compile_options(livox_perception_node PRIVATE -Wall -Wextra -Wpedantic)

ament_target_dependencies(livox_perception_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  livox_ros_driver2
)

install(TARGETS
  livox_perception_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  livox_perception_cuda
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
