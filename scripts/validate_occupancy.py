#!/usr/bin/env python3
"""
Occupancy Grid坐标系验证工具

此脚本订阅occupancy grid和点云，进行以下检查：
1. 验证occupancy的frame_id
2. 统计占据率
3. 检查点云与occupancy的空间对齐
4. 生成诊断报告
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from collections import deque
import time


class OccupancyValidator(Node):
    def __init__(self):
        super().__init__('occupancy_validator')
        
        # 订阅
        self.occ_sub = self.create_subscription(
            OccupancyGrid,
            '/livox/occupancy',
            self.occupancy_callback,
            10)
        
        self.lidar_pc_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar_filtered',
            self.lidar_pointcloud_callback,
            10)
        
        self.camera_pc_sub = self.create_subscription(
            PointCloud2,
            '/livox/camera_points',
            self.camera_pointcloud_callback,
            10)
        
        # 数据缓存
        self.latest_occupancy = None
        self.latest_lidar_pc = None
        self.latest_camera_pc = None
        
        # 统计数据
        self.occupancy_stats = deque(maxlen=100)
        self.point_count_stats = deque(maxlen=100)
        
        # 定时器：每5秒输出一次报告
        self.timer = self.create_timer(5.0, self.print_report)
        
        self.get_logger().info('Occupancy Validator started')
        self.get_logger().info('Waiting for data...')
    
    def occupancy_callback(self, msg):
        """处理occupancy grid消息"""
        self.latest_occupancy = msg
        
        # 统计占据格子
        data = np.array(msg.data, dtype=np.int8)
        occupied = np.sum(data == 100)
        free = np.sum(data == 0)
        unknown = np.sum(data == -1)
        total = len(data)
        
        stats = {
            'timestamp': time.time(),
            'frame_id': msg.header.frame_id,
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'origin_z': msg.info.origin.position.z,
            'orientation_w': msg.info.origin.orientation.w,
            'orientation_x': msg.info.origin.orientation.x,
            'orientation_y': msg.info.origin.orientation.y,
            'orientation_z': msg.info.origin.orientation.z,
            'occupied': occupied,
            'free': free,
            'unknown': unknown,
            'total': total,
            'occupied_ratio': occupied / total * 100 if total > 0 else 0,
            'free_ratio': free / total * 100 if total > 0 else 0
        }
        
        self.occupancy_stats.append(stats)
    
    def lidar_pointcloud_callback(self, msg):
        """处理雷达坐标系点云"""
        self.latest_lidar_pc = msg
        
        # 统计点数
        count = 0
        for _ in pc2.read_points(msg, skip_nans=True):
            count += 1
        
        self.point_count_stats.append({
            'timestamp': time.time(),
            'lidar_points': count
        })
    
    def camera_pointcloud_callback(self, msg):
        """处理camera坐标系点云"""
        self.latest_camera_pc = msg
    
    def print_report(self):
        """打印诊断报告"""
        if not self.occupancy_stats:
            self.get_logger().warn('No occupancy data received yet')
            return
        
        # 获取最新数据
        latest = self.occupancy_stats[-1]
        
        print("\n" + "="*80)
        print("OCCUPANCY GRID VALIDATION REPORT")
        print("="*80)
        
        # 1. 坐标系信息
        print("\n[1] COORDINATE FRAME")
        print(f"  Frame ID: {latest['frame_id']}")
        frame_ok = latest['frame_id'] == 'center_camera'
        status = "✅ CORRECT" if frame_ok else "❌ WRONG (should be 'center_camera')"
        print(f"  Status: {status}")
        
        # 2. 网格配置
        print("\n[2] GRID CONFIGURATION")
        print(f"  Resolution: {latest['resolution']:.3f} m/cell")
        print(f"  Grid Size: {latest['width']} x {latest['height']} cells")
        print(f"  Coverage: {latest['width'] * latest['resolution']:.1f}m x "
              f"{latest['height'] * latest['resolution']:.1f}m")
        
        # 3. 网格Origin（camera坐标系）
        print("\n[3] GRID ORIGIN (in camera frame)")
        print(f"  Position: x={latest['origin_x']:.3f}, "
              f"y={latest['origin_y']:.3f}, z={latest['origin_z']:.3f}")
        print(f"  Orientation: w={latest['orientation_w']:.3f}, "
              f"x={latest['orientation_x']:.3f}, "
              f"y={latest['orientation_y']:.3f}, "
              f"z={latest['orientation_z']:.3f}")
        
        # 4. 占据统计
        print("\n[4] OCCUPANCY STATISTICS")
        print(f"  Occupied cells: {latest['occupied']:,} ({latest['occupied_ratio']:.2f}%)")
        print(f"  Free cells: {latest['free']:,} ({latest['free_ratio']:.2f}%)")
        print(f"  Unknown cells: {latest['unknown']:,}")
        print(f"  Total cells: {latest['total']:,}")
        
        # 检查占据率是否合理
        occ_ratio = latest['occupied_ratio']
        if occ_ratio > 50:
            print(f"  ⚠️  WARNING: Occupancy ratio too high ({occ_ratio:.1f}%)!")
            print(f"     This might indicate coordinate system issues.")
        elif occ_ratio < 0.1:
            print(f"  ⚠️  WARNING: Occupancy ratio too low ({occ_ratio:.1f}%)!")
            print(f"     Check if sensor is receiving data.")
        else:
            print(f"  ✅ Occupancy ratio looks reasonable")
        
        # 5. 点云统计
        if self.point_count_stats:
            latest_pc = self.point_count_stats[-1]
            print("\n[5] POINT CLOUD STATISTICS")
            print(f"  Lidar points (filtered): {latest_pc.get('lidar_points', 'N/A'):,}")
            
            if self.latest_lidar_pc:
                print(f"  Lidar frame_id: {self.latest_lidar_pc.header.frame_id}")
            if self.latest_camera_pc:
                print(f"  Camera frame_id: {self.latest_camera_pc.header.frame_id}")
        
        # 6. 数据同步检查
        print("\n[6] DATA SYNCHRONIZATION")
        has_occ = self.latest_occupancy is not None
        has_lidar = self.latest_lidar_pc is not None
        has_camera = self.latest_camera_pc is not None
        
        print(f"  Occupancy Grid: {'✅ Received' if has_occ else '❌ Missing'}")
        print(f"  Lidar PointCloud: {'✅ Received' if has_lidar else '❌ Missing'}")
        print(f"  Camera PointCloud: {'✅ Received' if has_camera else '❌ Missing'}")
        
        # 7. 历史趋势（最近10个样本）
        if len(self.occupancy_stats) >= 2:
            recent = list(self.occupancy_stats)[-10:]
            avg_occ = np.mean([s['occupied_ratio'] for s in recent])
            std_occ = np.std([s['occupied_ratio'] for s in recent])
            
            print("\n[7] RECENT TREND (last 10 samples)")
            print(f"  Average occupancy: {avg_occ:.2f}% ± {std_occ:.2f}%")
            
            if std_occ > 10:
                print(f"  ⚠️  High variance detected - data might be unstable")
            else:
                print(f"  ✅ Stable occupancy pattern")
        
        # 8. 建议
        print("\n[8] RECOMMENDATIONS")
        if not frame_ok:
            print("  ❌ Frame ID is incorrect. Expected 'center_camera'")
            print("     Check the 'output_frame' parameter in config")
        
        if occ_ratio > 50:
            print("  ⚠️  High occupancy ratio detected:")
            print("     1. Check coordinate system transformation")
            print("     2. Verify grid_origin_x/y parameters")
            print("     3. Visualize in RViz with points overlay")
        
        if not (has_occ and has_lidar and has_camera):
            print("  ⚠️  Some data streams are missing:")
            print("     1. Check if livox_perception_node is running")
            print("     2. Verify topic names in launch file")
            print("     3. Check sensor connection")
        
        print("\n" + "="*80)
        print()


def main(args=None):
    rclpy.init(args=args)
    
    validator = OccupancyValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
