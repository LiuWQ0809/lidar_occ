#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${SCRIPT_DIR}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "[ERROR] ROS 2 distribution '${ROS_DISTRO}' is not installed under /opt/ros." >&2
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

cd "${WORKSPACE_ROOT}"

colcon build --symlink-install ${COLCON_BUILD_ARGS:-} \
  --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE:-Release} ${CMAKE_ARGS:-}
