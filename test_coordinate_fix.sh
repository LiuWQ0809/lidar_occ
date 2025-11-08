#!/bin/bash
# 快速测试脚本 - 验证坐标系修复

echo "======================================"
echo "Livox Perception 坐标系修复验证"
echo "======================================"
echo ""

# 进入工作目录
cd /home/nvidia/liuwq/lidar_perception

# 检查编译状态
echo "[1/5] 检查编译状态..."
if [ -d "install/livox_perception" ]; then
    echo "✅ 包已编译"
else
    echo "❌ 包未编译，开始编译..."
    colcon build --packages-select livox_perception --cmake-args -DCMAKE_BUILD_TYPE=Release
    if [ $? -ne 0 ]; then
        echo "❌ 编译失败！"
        exit 1
    fi
    echo "✅ 编译成功"
fi

# Source环境
echo ""
echo "[2/5] 配置环境..."
source install/setup.bash
echo "✅ 环境已配置"

# 检查ROS2话题（如果节点在运行）
echo ""
echo "[3/5] 检查ROS2话题..."
timeout 2 ros2 topic list > /tmp/ros2_topics.txt 2>&1
if grep -q "/livox/occupancy" /tmp/ros2_topics.txt; then
    echo "✅ 检测到 /livox/occupancy 话题"
    
    # 检查frame_id
    echo ""
    echo "[4/5] 验证frame_id..."
    FRAME_ID=$(timeout 3 ros2 topic echo /livox/occupancy --field header.frame_id --once 2>/dev/null | tr -d '[:space:]')
    if [ "$FRAME_ID" == "center_camera" ]; then
        echo "✅ Frame ID 正确: $FRAME_ID"
    else
        echo "⚠️  Frame ID: $FRAME_ID (预期: center_camera)"
    fi
    
    # 检查占据率
    echo ""
    echo "[5/5] 检查占据统计..."
    timeout 3 ros2 topic echo /livox/occupancy --once > /tmp/occ_msg.txt 2>/dev/null
    if [ -f /tmp/occ_msg.txt ]; then
        OCCUPIED=$(grep -o "100" /tmp/occ_msg.txt | wc -l)
        FREE=$(grep -o "^0$" /tmp/occ_msg.txt | wc -l)
        TOTAL=$(grep "^-\?[0-9]\+$" /tmp/occ_msg.txt | wc -l)
        
        if [ $TOTAL -gt 0 ]; then
            OCC_RATIO=$(echo "scale=2; $OCCUPIED * 100 / $TOTAL" | bc)
            echo "  占据格子: $OCCUPIED"
            echo "  空闲格子: $FREE"
            echo "  总格子数: $TOTAL"
            echo "  占据率: $OCC_RATIO%"
            
            if (( $(echo "$OCC_RATIO > 50" | bc -l) )); then
                echo "  ⚠️  警告: 占据率过高！可能存在坐标系问题"
            elif (( $(echo "$OCC_RATIO < 0.1" | bc -l) )); then
                echo "  ⚠️  警告: 占据率过低！检查传感器数据"
            else
                echo "  ✅ 占据率看起来合理"
            fi
        fi
    fi
else
    echo "⚠️  未检测到话题，节点可能未运行"
    echo ""
    echo "要启动节点，请运行:"
    echo "  ros2 launch livox_perception livox_perception.launch.py"
fi

# 清理临时文件
rm -f /tmp/ros2_topics.txt /tmp/occ_msg.txt

echo ""
echo "======================================"
echo "测试完成"
echo "======================================"
echo ""
echo "📋 详细文档: COORDINATE_FIX_README.md"
echo ""
echo "🔧 可用工具:"
echo "  1. 验证脚本: python3 scripts/validate_occupancy.py"
echo "  2. RViz配置: rviz2 -d config/occupancy_debug.rviz"
echo "  3. 日志查看: tail -f logs/livox_perception.log"
echo ""
