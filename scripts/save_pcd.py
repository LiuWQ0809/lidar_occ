#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg
import numpy as np
from sensor_msgs_py import point_cloud2
import os
import struct
from datetime import datetime

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        
        # 创建保存目录
        self.save_dir = './debug_pointclouds'
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 订阅原始Livox点云 (CustomMsg格式)
        self.raw_sub = self.create_subscription(
            CustomMsg,
            '/livox/lidar',
            self.raw_pointcloud_callback,
            10)
        
        # 订阅预处理后的点云 (PointCloud2格式)
        self.processed_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar_filtered',
            self.processed_pointcloud_callback,
            10)
        
        self.raw_saved = False
        self.processed_saved = False
        
        self.get_logger().info(f'点云保存节点已启动，保存目录: {self.save_dir}')
        self.get_logger().info('等待接收点云数据...')

    def custom_msg_to_array(self, custom_msg):
        """将Livox CustomMsg转换为numpy数组"""
        try:
            points_list = []
            for point in custom_msg.points:
                # Livox CustomPoint包含x, y, z, reflectivity, tag, line等字段
                points_list.append([point.x, point.y, point.z])
            
            if len(points_list) == 0:
                return None
                
            return np.array(points_list, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f'Livox点云数据转换失败: {e}')
            return None

    def pointcloud2_to_array(self, pointcloud_msg):
        """将PointCloud2消息转换为numpy数组"""
        try:
            points_list = []
            for point in point_cloud2.read_points(pointcloud_msg, skip_nans=True):
                points_list.append([point[0], point[1], point[2]])
            
            if len(points_list) == 0:
                return None
                
            return np.array(points_list, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f'PointCloud2数据转换失败: {e}')
            return None

    def save_pointcloud_as_pcd(self, points_array, filename):
        """保存点云数组为PCD文件"""
        try:
            if points_array is None or len(points_array) == 0:
                self.get_logger().warn(f'点云数据为空，跳过保存: {filename}')
                return False
            
            filepath = os.path.join(self.save_dir, filename)
            
            # 写入PCD文件头
            with open(filepath, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(points_array)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points_array)}\n")
                f.write("DATA ascii\n")
                
                # 写入点云数据
                for point in points_array:
                    f.write(f"{point[0]} {point[1]} {point[2]}\n")
            
            self.get_logger().info(f'成功保存点云: {filepath}')
            self.get_logger().info(f'点云包含 {len(points_array)} 个点')
            return True
            
        except Exception as e:
            self.get_logger().error(f'保存PCD文件失败: {e}')
            return False

    def raw_pointcloud_callback(self, msg):
        """原始Livox点云回调函数"""
        if self.raw_saved:
            return
            
        self.get_logger().info('接收到原始Livox点云数据，开始保存...')
        
        # 转换点云数据
        points_array = self.custom_msg_to_array(msg)
        
        # 生成文件名
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'raw_livox_pointcloud_{timestamp}.pcd'
        
        # 保存PCD文件
        if self.save_pointcloud_as_pcd(points_array, filename):
            self.raw_saved = True
            self.get_logger().info('原始Livox点云保存完成')
            self.check_completion()

    def processed_pointcloud_callback(self, msg):
        """预处理后点云回调函数"""
        if self.processed_saved:
            return
            
        self.get_logger().info('接收到预处理点云数据，开始保存...')
        
        # 转换点云数据
        points_array = self.pointcloud2_to_array(msg)
        
        # 生成文件名
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'processed_pointcloud_{timestamp}.pcd'
        
        # 保存PCD文件
        if self.save_pointcloud_as_pcd(points_array, filename):
            self.processed_saved = True
            self.get_logger().info('预处理点云保存完成')
            self.check_completion()

    def check_completion(self):
        """检查是否都保存完成"""
        if self.raw_saved and self.processed_saved:
            self.get_logger().info('所有点云文件保存完成，节点将退出')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PointCloudSaver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()