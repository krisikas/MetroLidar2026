#!/usr/bin/env bash
# ==============================================================================
# MetroLidar2026: Docker Automation Script for Autonomous Metro Stand
# Target Stand: Ubuntu 22.04 LTS, NVIDIA RTX 4070 Ti SUPER, CUDA 12.4, ROS 2 Humble
# ==============================================================================

set -eo pipefail

IMAGE_NAME="metrolidar2026:latest"
CONTAINER_NAME="metrolidar_detector_instance"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default parameters
LIDAR_TOPIC="/lidar_points"
TRAIN_SPEED="15.0"
QOS_RELIABILITY="sensor_data"
BAG_PATH=""
ACTION="run"
ENABLE_FOXGLOVE=true
USE_GPU=true

# Display help message
show_help() {
  cat << EOF
MetroLidar2026 - Production Docker Stand Runner
Usage: ./run_docker.sh [OPTIONS]

Options:
  -b, --build             Build / rebuild the Docker image
  --bag <path_or_name>    Auto-play the specified rosbag2 dataset (e.g. doubleT_obstacle)
  --topic <topic_name>    LiDAR PointCloud2 topic (default: /lidar_points)
  --speed <speed_mps>     Initial train speed in m/s (default: 15.0 m/s ~ 54 km/h)
  --foxglove              Also launch foxglove_bridge on port 8765
  --shell                 Drop into interactive bash shell inside container
  --test                  Execute automated unit tests (colcon test) inside container
  --no-gpu                Disable NVIDIA GPU acceleration flag
  -h, --help              Show this help message

Examples:
  ./run_docker.sh --build
  ./run_docker.sh --bag doubleT_obstacle
  ./run_docker.sh --bag roundT_doubleT --speed 20.0
  ./run_docker.sh --shell
EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    -b|--build)
      ACTION="build"
      shift
      ;;
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    --topic)
      LIDAR_TOPIC="$2"
      shift 2
      ;;
    --speed)
      TRAIN_SPEED="$2"
      shift 2
      ;;
    --foxglove)
      ENABLE_FOXGLOVE=true
      shift
      ;;
    --shell)
      ACTION="shell"
      shift
      ;;
    --test)
      ACTION="test"
      shift
      ;;
    --no-gpu)
      USE_GPU=false
      shift
      ;;
    -h|--help)
      show_help
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      show_help
      exit 1
      ;;
  esac
done

# If a bag name was given without full path, resolve it in archive/
if [[ -n "${BAG_PATH}" ]]; then
  if [[ ! -d "${BAG_PATH}" ]]; then
    POSSIBLE_PATH="${WORKSPACE_DIR}/archive/for_hackathon/${BAG_PATH}"
    if [[ -d "${POSSIBLE_PATH}" ]]; then
      BAG_PATH="${POSSIBLE_PATH}"
    else
      POSSIBLE_PATH2="${WORKSPACE_DIR}/archive/${BAG_PATH}"
      if [[ -d "${POSSIBLE_PATH2}" ]]; then
        BAG_PATH="${POSSIBLE_PATH2}"
      fi
    fi
  fi

  # Auto-detect Hesai 128 / Livox topic in doubleT_obstacle bag
  if [[ "${BAG_PATH}" == *"doubleT_obstacle"* ]]; then
    LIDAR_TOPIC="/sensing/lidar/hesai128/pointcloud"
    echo "[MetroLidar] Auto-selected LiDAR topic for obstacle dataset: ${LIDAR_TOPIC}"
  fi
fi

# Build image if requested or if it does not exist
if [[ "${ACTION}" == "build" ]] || ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "=========================================================================="
  echo "Building MetroLidar2026 Production Docker Image: ${IMAGE_NAME}..."
  echo "=========================================================================="
  docker build --build-arg BUILD_DATE="$(date +%s)" -t "${IMAGE_NAME}" -f "${WORKSPACE_DIR}/docker/Dockerfile" "${WORKSPACE_DIR}"
  if [[ "${ACTION}" == "build" ]]; then
    echo "[MetroLidar] Build completed successfully!"
    exit 0
  fi
fi

# Configure GPU flags
GPU_FLAGS=""
if [[ "${USE_GPU}" == true ]]; then
  if command -v nvidia-smi >/dev/null 2>&1; then
    echo "[MetroLidar] NVIDIA GPU detected: $(nvidia-smi --query-gpu=name --format=csv,noheader | head -n1)"
    GPU_FLAGS="--gpus all"
  else
    echo "[MetroLidar] Notice: nvidia-smi not detected on host. Running without GPU acceleration."
  fi
fi

# Handle interactive shell mode
if [[ "${ACTION}" == "shell" ]]; then
  echo "[MetroLidar] Starting interactive shell inside ${IMAGE_NAME}..."
  exec docker run --rm -it \
    ${GPU_FLAGS} \
    --net=host \
    --ipc=host \
    --shm-size=4g \
    -v "${WORKSPACE_DIR}/archive:/ros2_ws/archive:ro" \
    -e ROS_DOMAIN_ID=0 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    "${IMAGE_NAME}" bash
fi

# Handle automated testing mode
if [[ "${ACTION}" == "test" ]]; then
  echo "[MetroLidar] Running automated test suite inside container..."
  exec docker run --rm \
    ${GPU_FLAGS} \
    --net=host \
    --ipc=host \
    --shm-size=4g \
    -e ROS_DOMAIN_ID=0 \
    "${IMAGE_NAME}" bash -c "colcon test --event-handlers console_cohesion+ && colcon test-result --verbose"
fi

# Standard run mode
echo "=========================================================================="
echo "Starting MetroLidar2026 Obstacle Detector Stand"
echo "LiDAR Topic:  ${LIDAR_TOPIC}"
echo "Train Speed:  ${TRAIN_SPEED} m/s (~ $(awk "BEGIN {print ${TRAIN_SPEED} * 3.6}") km/h)"
echo "QoS Profile:  ${QOS_RELIABILITY}"
if [[ -n "${BAG_PATH}" ]]; then
  echo "Replaying:    ${BAG_PATH}"
fi
echo "=========================================================================="

# Cleanup previous instance if running
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

CONTAINER_BAG_PATH=""
if [[ -n "${BAG_PATH}" ]]; then
  CONTAINER_BAG_PATH="/ros2_ws/archive/$(basename "${BAG_PATH}")"
  REL_PATH="${BAG_PATH#"${WORKSPACE_DIR}/archive/"}"
  if [[ "${REL_PATH}" != "${BAG_PATH}" ]]; then
    CONTAINER_BAG_PATH="/ros2_ws/archive/${REL_PATH}"
  fi
  echo "[MetroLidar] Auto-replay enabled for: ${CONTAINER_BAG_PATH}"
fi

INTERACTIVE_FLAG="-it"
if [ ! -t 0 ]; then
  INTERACTIVE_FLAG="-t"
fi

exec docker run --rm ${INTERACTIVE_FLAG} \
  --name "${CONTAINER_NAME}" \
  ${GPU_FLAGS} \
  -p 8080:8080 \
  -p 8765:8765 \
  --ipc=host \
  --shm-size=4g \
  -v "${WORKSPACE_DIR}/archive:/ros2_ws/archive:ro" \
  -v "${WORKSPACE_DIR}/docker/cyclonedds.xml:/ros2_ws/cyclonedds.xml:ro" \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///ros2_ws/cyclonedds.xml \
  "${IMAGE_NAME}" \
  ros2 launch metro_obstacle_detector obstacle_detector.launch.py \
    lidar_topic:="${LIDAR_TOPIC}" \
    qos_reliability:="${QOS_RELIABILITY}" \
    train_speed_mps:="${TRAIN_SPEED}" \
    bag_path:="${CONTAINER_BAG_PATH}" \
    launch_web_gui:=true \
    launch_foxglove:="${ENABLE_FOXGLOVE}"
