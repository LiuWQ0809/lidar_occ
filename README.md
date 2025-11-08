# Livox Mid-360 占据栅格感知工程

该工程在 Jetson Orin AGX (JetPack 6.2) 平台上针对 Livox Mid-360 激光雷达构建实时占据栅格地图输出，输入 ROS2 topic 为 `/livox/lidar` 的点云。项目使用 C++ (Google style) 实现，基于 ROS2 Humble/Foxy + colcon，结合 CPU 版 PCL 预处理与 CUDA 占据栅格融合，以 tracking 模块稳定输出。

## 功能概述
- **点云预处理**：利用 PCL 对点云进行 NaN 清理、盲区剔除、空间裁剪与体素栅格降采样，遵循感知范围前/后 15 m、左右 3 m。并排除雷达右侧 0.5 m、前 2 cm、后 0.5 m 的矩形无效区。
- **GPU 占据融合**：针对可并行化的栅格占据步骤实现 CUDA kernel，将点云快速映射为 2D Occupancy Grid (nav_msgs/OccupancyGrid)。
- **历史跟踪**：通过概率式 GridTracker 对多帧结果进行时序平滑，抑制偶发噪点，保证输出稳定。
- **参数化配置**：所有核心参数（区域范围、无效区、体素尺寸、跟踪权重等）在 `config/livox_perception.yaml` 中集中管理，可按需求调整。
- **相机坐标输出**：内置 Livox→Camera 外参矩阵，将占据结果转换至中心相机坐标系便于后续感知融合。
- **RViz/Web 可视化**：默认提供 RViz2 视图查看点云与占据栅格，亦保留浏览器页面以便远程查看。
- **本地日志**：节点将运行统计写入项目目录下的 `logs/livox_perception.log`，便于排查与复现。
- **点云回发**：将过滤后的点云重新发布为 `sensor_msgs/PointCloud2`，方便 RViz / Web 查看。

## 目录结构
```
lidar_perception/
├── rviz                              # RViz2 配置文件
├── web                               # 浏览器可视化静态页面
└── src/livox_perception
    ├── include/livox_perception      # 预处理、栅格、追踪、CUDA 接口头文件
    ├── src                           # C++ 源码实现
    ├── cuda                          # 占据融合 CUDA kernel
    ├── config/livox_perception.yaml  # 默认参数
    ├── launch/livox_perception.launch.py
    ├── CMakeLists.txt
    └── package.xml
```

## 构建步骤
1. 准备 ROS2 交叉编译/运行环境，确保安装 `pcl`, `cuda-toolkit`, `eigen`, `ros-humble-nav-msgs` 等依赖。
2. 在 Orin 上启用 CUDA 与 Python 依赖：
   ```bash
   sudo apt install ros-humble-pcl-conversions ros-humble-tf2-geometry-msgs
   sudo apt install libpcl-dev nvidia-cuda-toolkit
   sudo apt install python3-pil
   ```
3. 构建工作区：
   ```bash
   cd /workdir/lidar_perception
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   ```
4. 运行前刷新环境：
   ```bash
   source install/setup.bash
   ros2 launch livox_perception livox_perception.launch.py
   ```

## 运行参数
- `lidar_topic`：默认 `/livox/lidar`。
- `/livox/lidar` Topic 类型为 `livox_ros_driver2/msg/CustomMsg`，来自 Livox ROS Driver 2。
- `grid_resolution`：默认 0.1 m，覆盖前后 30 m × 左右 6 m 区域。
- `invalid_rect_front/back/right/left`：Livox 自车保护区定义，默认匹配 Mid-360 安装要求。
- `tracker_*`：时序平滑参数，可按场景调整响应速度与抑噪能力。
- `lidar_to_camera_extrinsic`：16 个浮点数，按行优先排列的 4×4 外参矩阵，默认即题述标定文件，可根据实车重新标定后更新。
- `log_directory`：相对于项目根目录的日志输出文件夹，默认 `logs`。
- `pointcloud_topic`：二次发布的点云 Topic（已转换至相机坐标系），默认 `/livox/camera_points`。
- `min_height/max_height`：点云高度过滤范围，默认仅保留 [-0.3 m, 1.7 m] 内的目标。

## RViz2 可视化
1. 启动 Livox 感知节点（确保 `/livox/lidar` 有数据）：
   ```bash
   cd /workdir/lidar_perception
   ./run_livox_perception.sh
   ```
2. 另开终端 source 环境后启动 RViz2 预置配置：
   ```bash
   cd /workdir/lidar_perception
   source install/setup.bash
   rviz2 -d rviz/livox_perception.rviz
   ```
3. 在 RViz2 中可同时查看 `/livox/camera_points` 点云与 `/livox/occupancy` 占据栅格，基准坐标系为 `center_camera`，可使用鼠标拖拽、滚轮缩放调整视角。

> 若需继续使用浏览器版本，可参考 `web/` 下的页面配合 rosbridge，但推荐在调试阶段以 RViz2 为主。

## 占据栅格图像导出
1. 确认节点正在发布 `/livox/occupancy`。
2. 直接使用 Python 运行导出脚本（可选参数 `--save-directory`, `--image-format` 等）：
   ```bash
   cd /workdir/lidar_perception
   source install/setup.bash
   python3 src/livox_perception/scripts/occupancy_bev_export.py --save-directory occupancy_images --image-format jpg
   ```
3. 默认将以 BEV 视角保存 `occupancy_images/` 下的 JPG 文件，`--ros-args -p` 可进一步调整颜色、目录等参数。

## 模块说明
- **Preprocessor**：纯 CPU (PCL) 实现的点云裁剪与降采样，遵守 Livox Mid-360 盲区 (0.1 m) 和 FOV 特性，保证输入数据稳定。
- **CudaOccupancyIntegrator**：利用 CUDA 并行将点云投影至二维栅格，可充分利用 Orin GPU。支持自动显存管理，必要时可替换为 CPU 版本以做单元测试。
- **GridTracker**：帧间概率积累 + 衰减机制，控制占据阈值，减少瞬时噪点对结果的影响。

## 调试建议
- 检查 `/livox/lidar` 是否发布 `sensor_msgs/PointCloud2`；必要时使用 `ros2 topic echo` 或 RViz 验证。
- 若需 CPU-only 调试，可在 CMake 中去掉 CUDA 库并在 `GridMapper` 中直接使用 CPU 累积逻辑。
- 对于不同安装高度，可通过参数调整 `min_height/max_height` 与 map 原点提升对地面/障碍的过滤正确性。

## 后续扩展
- 添加射线追踪（Ray Tracing）以标注自由空间。
- 引入动态物体检测与分类，为轨迹规划提供语义信息。
- 与车辆姿态融合 (e.g., IMU/ODOM) 以实现全局地图对齐或滑动窗口拼接。
