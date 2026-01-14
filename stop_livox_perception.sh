#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${SCRIPT_DIR}"
cd "${WORKSPACE_ROOT}"
PROCESS_PATTERN="livox_perception"
PID_FILE="${WORKSPACE_ROOT}/.livox_launch.pid"

echo "========================================"
echo "Stopping Livox Perception..."
echo "========================================"
echo ""

# 先尝试使用保存的PID文件
if [ -f "$PID_FILE" ]; then
    SAVED_PID=$(cat "$PID_FILE" 2>/dev/null || true)
    if [ ! -z "$SAVED_PID" ] && kill -0 "$SAVED_PID" 2>/dev/null; then
        echo "Killing process with saved PID: $SAVED_PID"
        kill -9 "$SAVED_PID" 2>/dev/null || true
        rm -f "$PID_FILE"
        sleep 1
    fi
fi

# 再通过进程名查找任何残留进程
EXISTING_PIDS=$(pgrep -f "${PROCESS_PATTERN}" || true)
if [ ! -z "$EXISTING_PIDS" ]; then
    echo "Found remaining Livox Perception process(es):"
    echo "$EXISTING_PIDS"
    echo "Killing all remaining processes..."
    echo "$EXISTING_PIDS" | xargs kill -9 2>/dev/null || true
    sleep 1
fi

# 最后验证进程是否都已被清理
if pgrep -f "${PROCESS_PATTERN}" > /dev/null 2>&1; then
    echo "Warning: Some processes may still be running. Please check manually."
    pgrep -f "${PROCESS_PATTERN}" || true
else
    echo "Successfully stopped all Livox Perception processes."
fi

echo ""
echo "========================================"
