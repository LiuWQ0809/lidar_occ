# Livox Perception 编译问题优化方案

## 🐛 当前问题

```
fatal error: opencv2/core/core.hpp: No such file or directory
```

### 根本原因

1. **CMakeLists.txt 没有声明 OpenCV 依赖**
   - 源代码中使用了 OpenCV（14-15行）
   - 但 CMakeLists.txt 中没有 `find_package(OpenCV REQUIRED)`
   - 导致编译器找不到头文件路径

2. **package.xml 中也没有 OpenCV 依赖**
   - 缺少 `<depend>opencv</depend>` 或 `<build_depend>OpenCV</build_depend>`

---

## 🔧 解决方案

### 方案 A：添加 OpenCV 依赖（推荐）

#### 1. 修改 `CMakeLists.txt`

**在第 20 行后添加：**
```cmake
find_package(OpenCV REQUIRED)
```

**在 `include_directories` 中添加：**
```cmake
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}  # ← 添加这行
)
```

**在 `target_link_libraries` 中添加：**
```cpp
target_link_libraries(livox_perception_node
  livox_perception_cuda
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}  # ← 添加这行
)
```

#### 2. 修改 `package.xml`

**在 `<build_depend>` 部分添加：**
```xml
<build_depend>opencv</build_depend>
```

**在 `<exec_depend>` 部分添加：**
```xml
<exec_depend>opencv</exec_depend>
```

### 方案 B：删除不必要的 OpenCV 依赖

查看代码中 OpenCV 的实际用法：

```cpp
#include <opencv2/core/core.hpp>      // 14行
#include <opencv2/core/persistence.hpp> // 15行
```

搜索实际使用的地方...

（需要检查代码是否真正使用了 OpenCV）

---

## 📋 检查清单

### 步骤 1：验证 OpenCV 是否安装

```bash
# 检查 OpenCV 是否可用
pkg-config --modversion opencv4
# 或
find /usr -name opencv2 2>/dev/null | head -5
```

### 步骤 2：修复编译

```bash
# 清除旧的构建
rm -rf /home/nvidia/liuwq/lidar_occ/build
rm -rf /home/nvidia/liuwq/lidar_occ/install

# 重新构建
cd /home/nvidia/liuwq/lidar_occ
colcon build --symlink-install
```

### 步骤 3：检查依赖是否正确解析

```bash
# 检查 ROS 2 能否找到 OpenCV
ros2 pkg prefix opencv
```

---

## 🎯 关键代码修改

### CMakeLists.txt 的完整修改

```cmake
# 第 20 行后添加
find_package(OpenCV REQUIRED)

# 第 24-29 行修改为：
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# 第 60-64 行修改为：
target_link_libraries(livox_perception_node
  livox_perception_cuda
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
```

### package.xml 的完整修改

```xml
<!-- 在 <build_depend> 部分添加 -->
<build_depend>opencv</build_depend>

<!-- 在 <exec_depend> 部分添加 -->
<exec_depend>opencv</exec_depend>
```

---

## ⚠️ 备选方案：不使用 OpenCV

如果 OpenCV 的使用只是加载标定文件，可以考虑用纯 Eigen 或其他方式替代：

```cpp
// 当前代码（需要 OpenCV）
cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
cv::Mat tbc_mat;
fs["Tbc"] >> tbc_mat;

// 替代方案 1：使用 YAML 解析库（nlohmann/json）
// 替代方案 2：使用 Eigen 直接读取文本格式的矩阵
```

---

## 📊 对比表

| 方案 | 优点 | 缺点 |
|------|------|------|
| **方案 A：添加 OpenCV** | 保持代码不变，简单快速 | 增加依赖 |
| **方案 B：删除 OpenCV** | 减少依赖 | 需要重写代码 |

**推荐：方案 A**（因为改动最少）

---

## 🚀 快速修复步骤

```bash
# 1. 修改 CMakeLists.txt
cd /home/nvidia/liuwq/lidar_occ

# 2. 使用 sed 添加 find_package(OpenCV REQUIRED)
# （参考下面的具体修改）

# 3. 修改 package.xml
# （参考下面的具体修改）

# 4. 重新构建
colcon build --symlink-install --packages-select livox_perception

# 5. 启动节点
./run_livox_perception.sh
```

---

## 🔍 调试命令

```bash
# 显示详细的编译输出
colcon build --packages-select livox_perception --event-handlers console_direct+

# 检查 CMake 的查找过程
cmake -DCMAKE_MESSAGE_LOG_LEVEL=DEBUG ..

# 检查依赖是否正确解析
rosdep resolve opencv
```
