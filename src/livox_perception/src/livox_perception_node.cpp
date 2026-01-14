#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <ctime>

#include "Eigen/Dense"
#include <opencv2/core/core.hpp>
#include <opencv2/core/persistence.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/transforms.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include <filesystem>

#include "livox_perception/cuda_occupancy_integrator.hpp"
#include "livox_perception/grid_mapper.hpp"
#include "livox_perception/grid_tracker.hpp"
#include "livox_perception/preprocessor.hpp"

namespace livox_perception {

class LivoxPerceptionNode : public rclcpp::Node {
 public:
  LivoxPerceptionNode() : Node("livox_perception_node") {
    lidar_topic_ = this->declare_parameter<std::string>("lidar_topic", "/iv_points");
    output_frame_ = this->declare_parameter<std::string>("output_frame", "center_camera");
    grid_params_.resolution = this->declare_parameter("grid_resolution", 0.1);
    grid_params_.width_m = this->declare_parameter("grid_width", 30.0);
    grid_params_.height_m = this->declare_parameter("grid_height", 10.0);  // 修改：左右保留5米，总宽度10米
    grid_params_.origin_x = this->declare_parameter("grid_origin_x", -15.0);
    grid_params_.origin_y = this->declare_parameter("grid_origin_y", -5.0);  // 修改：原点偏移-5米
    grid_params_.origin_z = this->declare_parameter("grid_origin_z", -2.0);

    PreprocessorConfig preproc_config;
    preproc_config.min_distance = this->declare_parameter("min_distance", 0.1);
    preproc_config.max_forward = this->declare_parameter("max_forward", 15.0);
    preproc_config.max_backward = this->declare_parameter("max_backward", 15.0);
    preproc_config.max_left = this->declare_parameter("max_left", 5.0);  // 修改：左侧保留5米
    preproc_config.max_right = this->declare_parameter("max_right", 5.0);  // 修改：右侧保留5米
    preproc_config.min_height = this->declare_parameter("min_height", -0.3);
    preproc_config.max_height = this->declare_parameter("max_height", 1.8);  // 修改：高度上限改为1.8米
    preproc_config.invalid_rect_front = this->declare_parameter("invalid_rect_front", 0.02);
    preproc_config.invalid_rect_back = this->declare_parameter("invalid_rect_back", 0.5);
    preproc_config.invalid_rect_right = this->declare_parameter("invalid_rect_right", 0.5);
    preproc_config.invalid_rect_left = this->declare_parameter("invalid_rect_left", 0.0);
    preproc_config.voxel_leaf_size = this->declare_parameter("voxel_leaf_size", 0.1);

    TrackerConfig tracker_config;
    tracker_config.occupancy_increase_rate = this->declare_parameter("tracker_occupancy_increase", 0.7);
    tracker_config.decay_rate = this->declare_parameter("tracker_decay", 0.05);
    tracker_config.min_probability = this->declare_parameter("tracker_min_probability", 0.1);
    tracker_config.max_probability = this->declare_parameter("tracker_max_probability", 0.98);
    tracker_config.occupied_threshold = this->declare_parameter("tracker_occupied_threshold", 0.65);

    pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "/livox/camera_points");
    // 新增：雷达坐标系点云topic
    lidar_pointcloud_topic_ = this->declare_parameter<std::string>("lidar_pointcloud_topic", "/livox/lidar_filtered");
    
    const std::string log_directory_param =
        this->declare_parameter<std::string>("log_directory", "logs");

    const std::vector<double> default_extrinsic = {
        0.0087896171295762555,  -0.99995767027190419, -0.0026622674480113479,
        -0.20499999999999999,   0.00093490604571245953, 0.0026705879971036925,
        -0.99999607642824218,   1.2050000000000001,     0.99996118533156964,
        0.0087870551846044812,  0.00095834013107244957, 0.14999999999999999,
        0.0,                    0.0,                    0.0,
        1.0};
    auto lidar_to_camera_values =
        this->declare_parameter("lidar_to_camera_extrinsic", default_extrinsic);
    if (lidar_to_camera_values.size() != 16U) {
      throw std::runtime_error("lidar_to_camera_extrinsic must contain 16 values");
    }
    lidar_to_camera_.setIdentity();
    for (int idx = 0; idx < 16; ++idx) {
      lidar_to_camera_(idx / 4, idx % 4) = static_cast<float>(lidar_to_camera_values[idx]);
    }

    // 加载camera到body的转换矩阵
    const std::string calibration_file_param =
        this->declare_parameter<std::string>("calibration_file", 
            "calibration/front_left.yaml");
    // 如果是相对路径，相对于当前工作目录拼接完整路径
    std::string calibration_file = calibration_file_param;
    if (calibration_file.front() != '/') {
      std::filesystem::path cwd = std::filesystem::current_path();
      calibration_file = (cwd / calibration_file_param).string();
    }
    LoadCameraToBodyTransform(calibration_file);

    cuda_integrator_ = std::make_shared<CudaOccupancyIntegrator>();
    preprocessor_ = std::make_unique<Preprocessor>(preproc_config);
    grid_mapper_ = std::make_unique<GridMapper>(grid_params_, cuda_integrator_);
    tracker_ = std::make_unique<GridTracker>(tracker_config);

    occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/livox/occupancy", 10);
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 10);
    // 新增：雷达坐标系点云发布器
    lidar_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_pointcloud_topic_, 10);

    InitializeLogFile(log_directory_param);

    rclcpp::QoS qos(5);
    qos.reliable().durability_volatile();
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, qos,
        std::bind(&LivoxPerceptionNode::OnPointCloud, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Livox perception node initialized. Listening to %s", lidar_topic_.c_str());
  }

 private:
  void OnPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto lidar_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *lidar_cloud);

    // 修改：在雷达坐标系下进行预处理和建图
    auto filtered_lidar = preprocessor_->Filter(lidar_cloud);
    
    // 发布雷达坐标系的过滤点云（用于调试和可视化）
    sensor_msgs::msg::PointCloud2 lidar_pointcloud_msg;
    pcl::toROSMsg(*filtered_lidar, lidar_pointcloud_msg);
    lidar_pointcloud_msg.header.frame_id = msg->header.frame_id;  // 使用原始雷达帧ID
    lidar_pointcloud_msg.header.stamp = msg->header.stamp;
    lidar_pointcloud_pub_->publish(lidar_pointcloud_msg);

    // 关键修改：在雷达坐标系下建立occupancy网格
    auto grid = grid_mapper_->BuildOccupancyGrid(filtered_lidar);
    auto tracked = tracker_->Update(grid);

    // 修改：创建occupancy消息时，需要将网格的origin变换到camera坐标系
    auto occupancy_msg = grid_mapper_->CreateOccupancyGridMsg(msg->header.stamp, output_frame_, lidar_to_camera_);
    occupancy_msg.data = std::move(tracked);

    const auto occupied = static_cast<std::size_t>(std::count(occupancy_msg.data.begin(), occupancy_msg.data.end(), 100));
    const auto free = static_cast<std::size_t>(std::count(occupancy_msg.data.begin(), occupancy_msg.data.end(), 0));

    // 将occupancy grid的origin从camera坐标系转换到body坐标系
    Eigen::Vector3f origin_camera(occupancy_msg.info.origin.position.x,
                                   occupancy_msg.info.origin.position.y,
                                   occupancy_msg.info.origin.position.z);
    Eigen::Vector4f origin_camera_h(origin_camera.x(), origin_camera.y(), origin_camera.z(), 1.0f);
    Eigen::Vector4f origin_body_h = camera_to_body_ * origin_camera_h;
    
    occupancy_msg.info.origin.position.x = origin_body_h.x();
    occupancy_msg.info.origin.position.y = origin_body_h.y();
    occupancy_msg.info.origin.position.z = origin_body_h.z();
    
    // 更新frame_id为body坐标系
    occupancy_msg.header.frame_id = "body";

    // 为了可视化对比，也发布camera坐标系的点云
    auto camera_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::transformPointCloud(*filtered_lidar, *camera_cloud, lidar_to_camera_);
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*camera_cloud, pointcloud_msg);
    pointcloud_msg.header.frame_id = output_frame_;
    pointcloud_msg.header.stamp = msg->header.stamp;
    pointcloud_pub_->publish(pointcloud_msg);

    WriteLogEntry(msg->header, lidar_cloud->size(), filtered_lidar->size(), occupied, free);

    occupancy_pub_->publish(std::move(occupancy_msg));
  }

  std::string lidar_topic_;
  std::string output_frame_;
  std::string pointcloud_topic_;
  std::string lidar_pointcloud_topic_;  // 新增成员变量
  GridParameters grid_params_;

  Eigen::Matrix4f lidar_to_camera_;
  Eigen::Matrix4f camera_to_body_;  // camera到body的转换矩阵

  std::shared_ptr<CudaOccupancyIntegrator> cuda_integrator_;
  std::unique_ptr<Preprocessor> preprocessor_;
  std::unique_ptr<GridMapper> grid_mapper_;
  std::unique_ptr<GridTracker> tracker_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pointcloud_pub_;  // 新增发布器

  void LoadCameraToBodyTransform(const std::string & calibration_file) {
    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file: %s", calibration_file.c_str());
      throw std::runtime_error("Cannot open calibration file");
    }

    cv::Mat tbc_mat;
    fs["Tbc"] >> tbc_mat;
    fs.release();

    if (tbc_mat.empty() || tbc_mat.rows != 4 || tbc_mat.cols != 4) {
      RCLCPP_ERROR(this->get_logger(), "Invalid Tbc matrix in calibration file");
      throw std::runtime_error("Invalid Tbc matrix");
    }

    // 将OpenCV Mat转换为Eigen Matrix4f
    camera_to_body_.setIdentity();
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        camera_to_body_(i, j) = tbc_mat.at<float>(i, j);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded camera_to_body transform from %s", calibration_file.c_str());
  }

  void InitializeLogFile(const std::string & log_directory_param) {
    namespace fs = std::filesystem;
    fs::path log_dir(log_directory_param);
    if (!log_dir.is_absolute()) {
      log_dir = fs::current_path() / log_dir;
    }

    std::error_code ec;
    fs::create_directories(log_dir, ec);
    if (ec) {
      RCLCPP_WARN(this->get_logger(), "Failed to create log directory %s: %s", log_dir.string().c_str(), ec.message().c_str());
    }

    log_file_path_ = log_dir / "livox_perception.log";
    log_stream_.open(log_file_path_, std::ios::out | std::ios::app);
    if (!log_stream_.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Unable to open log file %s", log_file_path_.string().c_str());
    } else {
      log_stream_ << "\n==== Livox Perception Log Start ====" << std::endl;
      log_stream_.flush();
      RCLCPP_INFO(this->get_logger(), "Logging to %s", log_file_path_.string().c_str());
    }
  }

  void WriteLogEntry(const std_msgs::msg::Header & header, std::size_t raw_points,
                     std::size_t filtered_points, std::size_t occupied_cells,
                     std::size_t free_cells) {
    if (!log_stream_.is_open()) {
      return;
    }

    const auto now = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_time = *std::localtime(&now_time);

    log_stream_ << std::put_time(&tm_time, "%Y-%m-%d %H:%M:%S") << '.'
                << std::setfill('0') << std::setw(3) << (ms.count() % 1000)
                << " frame_id=" << header.frame_id
                << " stamp=" << header.stamp.sec << '.'
                << std::setw(9) << header.stamp.nanosec << std::setfill(' ')
                << " raw_points=" << raw_points
                << " filtered_points=" << filtered_points
                << " occupied_cells=" << occupied_cells
                << " free_cells=" << free_cells << std::endl;
    log_stream_.flush();
  }

  std::filesystem::path log_file_path_;
  std::ofstream log_stream_;
};

}  // namespace livox_perception

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<livox_perception::LivoxPerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
