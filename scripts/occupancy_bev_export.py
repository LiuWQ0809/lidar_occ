#!/usr/bin/env python3
"""Convert nav_msgs/OccupancyGrid into BEV image snapshots."""

import argparse
import os
import sys
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import OccupancyGrid

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("Pillow is required. Install via 'sudo apt install python3-pil'.") from exc


def get_text_size(draw, text, font):
    """获取文本尺寸的兼容函数"""
    try:
        # 尝试使用新版API
        bbox = draw.textbbox((0, 0), text, font=font)
        return bbox[2] - bbox[0], bbox[3] - bbox[1]
    except (ValueError, AttributeError):
        # 降级到旧版API或估算
        try:
            return draw.textsize(text, font=font)
        except AttributeError:
            # 如果都不支持，使用固定估算
            return len(text) * 7, 12


class OccupancyGridExporter(Node):
    """Subscribe to OccupancyGrid and export bird's-eye-view images."""

    def __init__(self, save_directory: str, image_format: str, overwrite: bool) -> None:
        super().__init__('occupancy_bev_exporter')

        self.save_dir = os.path.abspath(self.declare_parameter('save_directory', save_directory).get_parameter_value().string_value)
        self.image_format = self.declare_parameter('image_format', image_format).get_parameter_value().string_value.lower()
        self.overwrite = self.declare_parameter('overwrite', overwrite).get_parameter_value().bool_value
        self.topic = self.declare_parameter('occupancy_topic', '/livox/occupancy').get_parameter_value().string_value
        self.occupied_color = self._read_color_parameter('occupied_color', [255, 80, 80])
        self.free_color = self._read_color_parameter('free_color', [40, 40, 40])
        self.unknown_color = self._read_color_parameter('unknown_color', [128, 128, 128])
        self.pixels_per_cell = max(1, int(self.declare_parameter('pixels_per_cell', 6).get_parameter_value().integer_value))

        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"Saving occupancy snapshots to {self.save_dir} as *.{self.image_format}")

        self.subscription = self.create_subscription(OccupancyGrid, self.topic, self._on_grid, 10)
        self.frame_counter = 0

    def _read_color_parameter(self, name: str, default: list) -> tuple:
        value = list(self.declare_parameter(name, default).get_parameter_value().integer_array_value)
        if len(value) < 3:
            value.extend(default[len(value):3])
        return tuple(int(max(0, min(255, c))) for c in value[:3])

    def _on_grid(self, msg: OccupancyGrid) -> None:
        width = msg.info.width
        height = msg.info.height
        if width == 0 or height == 0:
            self.get_logger().warn('Received empty occupancy grid')
            return

        data = np.array(msg.data, dtype=np.int16).reshape((height, width))
        # 修复：occupancy grid数据是行优先存储(row-major)
        # data[y * width + x] 映射到 grid[y, x]
        # 需要转置以正确显示：grid[行, 列] -> BEV[Y方向, X方向]
        # 然后垂直翻转使前方(X+)朝上
        bev = np.flipud(data.T)

        # 修复：由于转置，BEV的尺寸是 (width, height) 而不是 (height, width)
        rgb = np.zeros((width, height, 3), dtype=np.uint8)
        rgb[bev == -1] = self.unknown_color
        rgb[bev == 0] = self.free_color
        rgb[bev > 0] = self.occupied_color

        image = Image.fromarray(rgb, mode='RGB')

        if self.pixels_per_cell > 1:
            # 修复：由于转置，新尺寸也需要调整
            new_size = (height * self.pixels_per_cell, width * self.pixels_per_cell)
            image = image.resize(new_size, resample=Image.NEAREST)
        draw = ImageDraw.Draw(image)

        # 修复：传递正确的宽高（转置后）
        self._draw_grid(draw, height, width)
        self._draw_axes(draw, msg)

        if self.overwrite:
            filename = os.path.join(self.save_dir, f"occupancy_bev.{self.image_format}")
        else:
            timestamp = datetime.fromtimestamp(self.get_clock().now().nanoseconds * 1e-9).strftime('%Y%m%d_%H%M%S_%f')
            filename = os.path.join(self.save_dir, f"occupancy_{timestamp}.{self.image_format}")

        try:
            image.save(filename)
            self.get_logger().info(f"Saved occupancy image to {filename}")
        except OSError as exc:  # pragma: no cover
            self.get_logger().error(f"Failed to save image to {filename}: {exc}")

        self.frame_counter += 1

    def _draw_grid(self, draw: ImageDraw.ImageDraw, width: int, height: int) -> None:
        if self.pixels_per_cell <= 1:
            return

        grid_color = (70, 70, 70)
        strong_color = (110, 110, 110)
        img_width = width * self.pixels_per_cell
        img_height = height * self.pixels_per_cell

        for col in range(width + 1):
            x = col * self.pixels_per_cell
            color = strong_color if (col % 5 == 0) else grid_color  # every 5 cells (~0.5 m @ 0.1 m resolution)
            draw.line([(x, 0), (x, img_height)], fill=color, width=1)

        for row in range(height + 1):
            y = row * self.pixels_per_cell
            color = strong_color if (row % 5 == 0) else grid_color
            draw.line([(0, y), (img_width, y)], fill=color, width=1)

    def _draw_axes(self, draw: ImageDraw.ImageDraw, msg: OccupancyGrid) -> None:
        """
        绘制坐标轴
        
        坐标系说明：
        - OccupancyGrid: 数据是行优先(row-major), data[y*width + x]
        - 可视化: 转置后BEV图像，X轴水平向右(前方)，Y轴垂直向上(左侧)
        - Camera坐标系: frame_id = center_camera
        """
        width = msg.info.width  # X方向格子数
        height = msg.info.height  # Y方向格子数
        res = msg.info.resolution
        
        # 由于转置，图像尺寸是 (height, width)
        img_width = height * self.pixels_per_cell   # 对应Y方向
        img_height = width * self.pixels_per_cell   # 对应X方向

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # 计算机器人坐标(0,0)在网格中的位置
        grid_x = int(round((0.0 - origin_x) / res))  # X方向的网格索引
        grid_y = int(round((0.0 - origin_y) / res))  # Y方向的网格索引

        if not (0 <= grid_x < width and 0 <= grid_y < height):
            if not hasattr(self, '_warned_origin_outside'):
                self.get_logger().warn('Robot origin (0,0) lies outside occupancy grid; axes may be clipped.')
                self._warned_origin_outside = True
            grid_x = max(0, min(width - 1, grid_x))
            grid_y = max(0, min(height - 1, grid_y))

        # 转置后的像素坐标
        # 原grid[grid_y, grid_x] -> 转置后BEV[grid_x, grid_y]
        # X方向(前方): 图像垂直方向，从下到上递增，需要flipud所以是从上到下
        # Y方向(左侧): 图像水平方向，从左到右递增
        origin_px_y = grid_y * self.pixels_per_cell + self.pixels_per_cell // 2  # Y在图像水平方向
        origin_px_x = (width - 1 - grid_x) * self.pixels_per_cell + self.pixels_per_cell // 2  # X在图像垂直方向(翻转)

        axis_color = (255, 255, 255)
        arrow_size = max(6, self.pixels_per_cell * 2)

        # X轴(前方): 垂直方向，向上为正
        draw.line([(origin_px_y, img_height - 1), (origin_px_y, 0)], fill=axis_color, width=2)
        draw.line([(origin_px_y - arrow_size // 2, arrow_size), (origin_px_y, 0),
                   (origin_px_y + arrow_size // 2, arrow_size)], fill=axis_color, width=2)

        # Y轴(左侧): 水平方向，向右为正
        draw.line([(0, origin_px_x), (img_width - 1, origin_px_x)], fill=axis_color, width=2)
        draw.line([(img_width - arrow_size, origin_px_x - arrow_size // 2), (img_width - 1, origin_px_x),
                   (img_width - arrow_size, origin_px_x + arrow_size // 2)], fill=axis_color, width=2)

        font = ImageFont.load_default()
        draw.text((origin_px_y + 5, 5), 'X (forward, m)', fill=axis_color, font=font)
        draw.text((img_width - 80, origin_px_x + 5), 'Y (left, m)', fill=axis_color, font=font)

        tick_cells = max(1, int(round(0.5 / res)))
        self._draw_axis_ticks(draw, grid_x, grid_y, width, height, tick_cells, res, font)

    def _draw_axis_ticks(self, draw: ImageDraw.ImageDraw, grid_x: int, grid_y: int,
                         width: int, height: int, tick_cells: int, resolution: float,
                         font: ImageFont.ImageFont) -> None:
        """
        绘制坐标轴刻度
        
        参数：
        - grid_x, grid_y: 机器人原点(0,0)在网格中的索引
        - width, height: 网格的宽高（X, Y方向）
        """
        axis_color = (230, 230, 230)
        tick_len = max(4, self.pixels_per_cell)
        
        # 转置后的像素坐标
        img_width = height * self.pixels_per_cell   # 对应Y方向
        img_height = width * self.pixels_per_cell   # 对应X方向
        
        origin_px_y = grid_y * self.pixels_per_cell + self.pixels_per_cell // 2
        origin_px_x = (width - 1 - grid_x) * self.pixels_per_cell + self.pixels_per_cell // 2

        # X轴刻度 (垂直方向，前后)
        for direction in (-1, 1):
            offset = tick_cells
            while True:
                gx = grid_x + direction * offset
                if gx < 0 or gx >= width:
                    break
                # 转置+翻转后的Y坐标
                py = (width - 1 - gx) * self.pixels_per_cell + self.pixels_per_cell // 2
                draw.line([(origin_px_y - tick_len, py), (origin_px_y + tick_len, py)], 
                         fill=axis_color, width=1)
                value_m = direction * offset * resolution
                label = f"{value_m:+.1f}"
                text_w, text_h = get_text_size(draw, label, font)
                draw.text((origin_px_y + tick_len + 2, py - text_h / 2), 
                         label, fill=axis_color, font=font)
                offset += tick_cells

        # Y轴刻度 (水平方向，左右)
        for direction in (-1, 1):
            offset = tick_cells
            while True:
                gy = grid_y + direction * offset
                if gy < 0 or gy >= height:
                    break
                # 转置后的X坐标
                px = gy * self.pixels_per_cell + self.pixels_per_cell // 2
                draw.line([(px, origin_px_x - tick_len), (px, origin_px_x + tick_len)], 
                         fill=axis_color, width=1)
                value_m = direction * offset * resolution
                label = f"{value_m:+.1f}"
                text_w, text_h = get_text_size(draw, label, font)
                draw.text((px - text_w / 2, origin_px_x + tick_len + 2),
                          label, fill=axis_color, font=font)
                offset += tick_cells

def main(argv=None) -> None:
    argv = argv if argv is not None else []
    parser = argparse.ArgumentParser(description='Export OccupancyGrid messages as BEV images', add_help=True)
    parser.add_argument('--save-directory', default='occupancy_images', help='Output directory for images')
    parser.add_argument('--image-format', default='jpg', choices=['jpg', 'png'], help='Image format to save')
    parser.add_argument('--overwrite', action='store_true', help='Overwrite the same filename instead of timestamping')

    non_ros_args = remove_ros_args(argv)
    options = parser.parse_args(non_ros_args[1:] if len(non_ros_args) > 1 else [])

    rclpy.init(args=argv)
    node = OccupancyGridExporter(options.save_directory, options.image_format, options.overwrite)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
