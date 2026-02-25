#!/bin/bash
# Build ROS 2 latency binary (works for both CycloneDDS and FastDDS rmw at runtime).
# Requires ROS 2 Humble installed at /opt/ros/humble.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${SCRIPT_DIR}/src/ros2_latency.cpp"
OUT_DIR="${SCRIPT_DIR}/bin"
BUILD_DIR="${SCRIPT_DIR}/build_ros2"
ROS_ROOT="${ROS_ROOT:-/opt/ros/humble}"

if [ ! -f "${ROS_ROOT}/setup.bash" ]; then
    echo "ERROR: ROS 2 not found at ${ROS_ROOT}"
    echo "  Set ROS_ROOT env var to your ROS 2 install prefix."
    exit 1
fi

mkdir -p "${OUT_DIR}" "${BUILD_DIR}"

# Generate a minimal CMakeLists.txt in the build dir
cat > "${BUILD_DIR}/CMakeLists.txt" << 'CMEOF'
cmake_minimum_required(VERSION 3.16)
project(ros2_latency)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
add_executable(lat_ros2 SRC_PLACEHOLDER)
ament_target_dependencies(lat_ros2 rclcpp std_msgs)
CMEOF

# Replace placeholder with actual source path
sed -i "s|SRC_PLACEHOLDER|${SRC}|g" "${BUILD_DIR}/CMakeLists.txt"

echo "=== Building ROS 2 latency binary ==="
# Temporarily disable nounset so ROS 2 setup.bash can use unset variables
set +u
source "${ROS_ROOT}/setup.bash"
set -u
cmake "${BUILD_DIR}" \
    -B "${BUILD_DIR}" \
    -DCMAKE_PREFIX_PATH="${ROS_ROOT}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DRMW_IMPLEMENTATION=rmw_cyclonedds_cpp 2>&1 | grep -E "Using RMW|Error|error" || true
cmake --build "${BUILD_DIR}" -j"$(nproc)"

cp "${BUILD_DIR}/lat_ros2" "${OUT_DIR}/lat_ros2"
echo "  -> ${OUT_DIR}/lat_ros2"
echo "  (use RMW_IMPLEMENTATION env var to select backend at runtime)"
echo ""
echo "Done."
