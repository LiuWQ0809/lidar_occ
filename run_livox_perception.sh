#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${SCRIPT_DIR}"
cd "${WORKSPACE_ROOT}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
PROCESS_PATTERN="livox_perception.launch"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "[ERROR] ROS 2 distribution '${ROS_DISTRO}' is not installed under /opt/ros." >&2
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if [ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
  echo "[ERROR] Workspace not built. Run ./build.sh first." >&2
  exit 1
fi

set +u
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

# 检查进程是否已启动，如果启动则kill掉
echo "========================================"
echo "Checking for existing Livox Perception processes..."
EXISTING_PIDS=$(pgrep -f "${PROCESS_PATTERN}" || true)
if [ ! -z "$EXISTING_PIDS" ]; then
    echo "Found existing process(es):"
    echo "$EXISTING_PIDS"
    echo "Killing existing processes..."
    echo "$EXISTING_PIDS" | xargs kill -9 2>/dev/null || true
    sleep 2
    
    # 验证进程是否已被kill
    if pgrep -f "${PROCESS_PATTERN}" > /dev/null 2>&1; then
        echo "Warning: Failed to kill existing processes. Some processes may still be running."
    else
        echo "Successfully killed existing processes."
    fi
fi

sleep 1

echo "Starting Livox Perception..."
echo "========================================"
echo ""

CONFIG_ARGUMENT=""
if [ $# -gt 0 ]; then
  CONFIG_ARGUMENT="config:=$1"
fi

# 运行节点，保存PID
ros2 launch livox_perception livox_perception.launch.py ${CONFIG_ARGUMENT} &
LAUNCH_PID=$!
echo "Launch process started with PID: $LAUNCH_PID"
echo $LAUNCH_PID > "${WORKSPACE_ROOT}/.livox_launch.pid"

# wait $LAUNCH_PID
