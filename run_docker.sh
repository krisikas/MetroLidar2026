#!/usr/bin/env bash
# ==============================================================================
# MetroLidar2026: Production Stand Automation Runner
# Environment: Ubuntu 22.04 LTS (NVIDIA RTX 4070 Ti / CPU), ROS 2 Humble
# Standard: GOST 9238 Clearance Envelope (Gauge "M")
# ==============================================================================

set -eo pipefail

# Determine repository root directory regardless of invocation path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "${SCRIPT_DIR}"

IMAGE_NAME="metrolidar2026:latest"
CONTAINER_NAME="metrolidar_detector_instance"
BAGS_DIR="${SCRIPT_DIR}/bags"
ARCHIVE_DIR="${SCRIPT_DIR}/archive"

# Default configuration parameters
LIDAR_TOPIC=""
TRAIN_SPEED="0.0"
QOS_RELIABILITY="sensor_data"
BAG_ARG=""
AUTORUN=false
ACTION="run"
ENABLE_FOXGLOVE=true
ENABLE_WEB_GUI=true
USE_GPU=true
DRY_RUN=false

# Display comprehensive help message
show_help() {
  cat << EOF
MetroLidar2026 - Autonomous Metro Stand Runner
Usage: bash run_docker.sh [OPTIONS]

Primary Options:
  --bag <name_or_path>   Run detection on specified rosbag (folder or .db3/.mcap file).
                         Looks in bags/, archive/for_hackathon/, or relative/absolute path.
  -a, --auto, --autorun  Auto-detect and run the first available bag from bags/ or archive/.
  -b, --build            Build / rebuild the Docker image (metrolidar2026:latest).

Configuration Options:
  --topic <topic_name>   LiDAR PointCloud2 topic (auto-detected from bag by default).
  --speed <speed_mps>    Initial train speed in m/s (default: 0.0 for dynamic LiDAR estimation).
  --qos <profile>        QoS reliability profile: sensor_data (default), best_effort, reliable.
  --no-gui               Disable Web GUI dashboard (port 8080).
  --no-foxglove          Disable Foxglove Studio WebSocket bridge (port 8765).
  --no-gpu               Disable NVIDIA GPU acceleration flag.

Diagnostics & Testing:
  --shell                Drop into interactive bash shell inside the container.
  --test                 Execute full automated test suite (colcon test + system verification).
  --dry-run              Print the resolved configuration and docker run command without executing.
  -h, --help             Show this help message and list available datasets.

Examples:
  # 1. Build image:
  bash run_docker.sh --build

  # 2. Auto-run first found dataset:
  bash run_docker.sh --auto

  # 3. Run specific dataset (drop into bags/ and pass name):
  bash run_docker.sh --bag doubleT_obstacle
  bash run_docker.sh --bag roundT_pressureGate_roundT

  # 4. Run automated tests inside container:
  bash run_docker.sh --test
EOF
}

# List all available datasets found in bags/ and archive/
list_available_bags() {
  echo "Available datasets found on host:"
  local count=0
  if [[ -d "${BAGS_DIR}" ]]; then
    for d in "${BAGS_DIR}"/*/; do
      if [[ -d "${d}" ]]; then
        local bname
        bname="$(basename "${d}")"
        echo "  * [bags/] ${bname}"
        count=$((count + 1))
      fi
    done
    for f in "${BAGS_DIR}"/*.db3 "${BAGS_DIR}"/*.mcap; do
      if [[ -f "${f}" ]]; then
        echo "  * [bags/] $(basename "${f}")"
        count=$((count + 1))
      fi
    done
  fi
  if [[ -d "${ARCHIVE_DIR}/for_hackathon" ]]; then
    for d in "${ARCHIVE_DIR}/for_hackathon"/*/; do
      if [[ -d "${d}" ]]; then
        echo "  * [archive/for_hackathon/] $(basename "${d}")"
        count=$((count + 1))
      fi
    done
  fi
  if [[ ${count} -eq 0 ]]; then
    echo "  (No datasets currently found in bags/ or archive/)"
  fi
}

# Resolve bag path from query string
resolve_bag() {
  local query="$1"
  local found=""

  # 1. Direct path check (as passed: relative to PWD or absolute)
  if [[ -e "${query}" ]]; then
    found="${query}"
  # 2. Check in bags/ directory
  elif [[ -e "${BAGS_DIR}/${query}" ]]; then
    found="${BAGS_DIR}/${query}"
  elif [[ -e "${BAGS_DIR}/${query}.db3" ]]; then
    found="${BAGS_DIR}/${query}.db3"
  elif [[ -e "${BAGS_DIR}/${query}.mcap" ]]; then
    found="${BAGS_DIR}/${query}.mcap"
  # 3. Check in archive/for_hackathon/ directory
  elif [[ -e "${ARCHIVE_DIR}/for_hackathon/${query}" ]]; then
    found="${ARCHIVE_DIR}/for_hackathon/${query}"
  # 4. Check in archive/ directory
  elif [[ -e "${ARCHIVE_DIR}/${query}" ]]; then
    found="${ARCHIVE_DIR}/${query}"
  fi

  if [[ -n "${found}" ]]; then
    if [[ -d "${found}" ]]; then
      (cd "${found}" && pwd)
    else
      local pdir
      pdir="$(cd "$(dirname "${found}")" && pwd)"
      echo "${pdir}/$(basename "${found}")"
    fi
  fi
}

# Auto-detect first available bag
find_first_bag() {
  # 1. Check bags/ folder
  if [[ -d "${BAGS_DIR}" ]]; then
    for d in "${BAGS_DIR}"/*/; do
      if [[ -d "${d}" ]]; then
        if [[ -f "${d}metadata.yaml" ]] || compgen -G "${d}*.db3" >/dev/null 2>&1 || compgen -G "${d}*.mcap" >/dev/null 2>&1; then
          (cd "${d}" && pwd)
          return 0
        fi
      fi
    done
    for f in "${BAGS_DIR}"/*.db3 "${BAGS_DIR}"/*.mcap; do
      if [[ -f "${f}" ]]; then
        echo "${f}"
        return 0
      fi
    done
  fi

  # 2. Check archive/for_hackathon/
  if [[ -d "${ARCHIVE_DIR}/for_hackathon" ]]; then
    for d in "${ARCHIVE_DIR}/for_hackathon"/*/; do
      if [[ -d "${d}" ]]; then
        if [[ -f "${d}metadata.yaml" ]] || compgen -G "${d}*.db3" >/dev/null 2>&1; then
          (cd "${d}" && pwd)
          return 0
        fi
      fi
    done
  fi

  # 3. Check archive/
  if [[ -d "${ARCHIVE_DIR}" ]]; then
    for d in "${ARCHIVE_DIR}"/*/; do
      if [[ -d "${d}" ]]; then
        if [[ -f "${d}metadata.yaml" ]] || compgen -G "${d}*.db3" >/dev/null 2>&1; then
          (cd "${d}" && pwd)
          return 0
        fi
      fi
    done
  fi

  return 1
}

# Auto-detect PointCloud2 topic from bag metadata or db3
detect_bag_topic() {
  local target="$1"
  local detected=""

  local meta=""
  if [[ -d "${target}" && -f "${target}/metadata.yaml" ]]; then
    meta="${target}/metadata.yaml"
  elif [[ -f "${target}" && -f "$(dirname "${target}")/metadata.yaml" ]]; then
    meta="$(dirname "${target}")/metadata.yaml"
  fi

  if [[ -n "${meta}" && -f "${meta}" ]]; then
    # Try fast pure-grep extraction first (zero python dependency)
    detected=$(grep -B 1 "sensor_msgs/msg/PointCloud2" "${meta}" 2>/dev/null | grep "name:" | head -n1 | awk '{print $NF}' | tr -d "\"'" || true)
    if [[ -z "${detected}" ]]; then
      detected=$(grep -A 1 "sensor_msgs/msg/PointCloud2" "${meta}" 2>/dev/null | grep "name:" | head -n1 | awk '{print $NF}' | tr -d "\"'" || true)
    fi
  fi

  if [[ -z "${detected}" ]]; then
    local db3=""
    if [[ -f "${target}" && "${target}" == *.db3 ]]; then
      db3="${target}"
    elif [[ -d "${target}" ]]; then
      db3=$(find "${target}" -maxdepth 1 -name "*.db3" 2>/dev/null | head -n1 || true)
    fi

    # If sqlite3 binary is present, query directly
    if [[ -n "${db3}" && -f "${db3}" ]]; then
      if command -v sqlite3 >/dev/null 2>&1; then
        detected=$(sqlite3 "${db3}" "SELECT name FROM topics WHERE type='sensor_msgs/msg/PointCloud2' LIMIT 1;" 2>/dev/null || true)
      fi
    fi
  fi

  if [[ -z "${detected}" ]]; then
    if [[ "${target}" == *"doubleT_obstacle"* ]]; then
      detected="/sensing/lidar/hesai128/pointcloud"
    else
      detected="/lidar_points"
    fi
  fi

  echo "${detected}"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    -b|--build)
      ACTION="build"
      shift
      ;;
    -a|--auto|--autorun)
      AUTORUN=true
      shift
      ;;
    --bag)
      BAG_ARG="$2"
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
    --qos)
      QOS_RELIABILITY="$2"
      shift 2
      ;;
    --no-gui)
      ENABLE_WEB_GUI=false
      shift
      ;;
    --no-foxglove)
      ENABLE_FOXGLOVE=false
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
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    -h|--help)
      show_help
      echo ""
      list_available_bags
      exit 0
      ;;
    *)
      echo "[MetroLidar] Error: Unknown option '$1'"
      show_help
      exit 1
      ;;
  esac
done

# If no arguments given at all, show help + available datasets
if [[ -z "${ACTION}" || "${ACTION}" == "run" ]] && [[ -z "${BAG_ARG}" && "${AUTORUN}" == false ]]; then
  echo "=========================================================================="
  echo "MetroLidar2026: Autonomous Obstacle Detection Stand"
  echo "=========================================================================="
  echo "Notice: No dataset specified. Use --auto to run the first available bag,"
  echo "or --bag <name> to run a specific dataset."
  echo ""
  list_available_bags
  echo ""
  echo "Quick start examples:"
  echo "  bash run_docker.sh --auto"
  echo "  bash run_docker.sh --bag doubleT_obstacle"
  echo "  bash run_docker.sh --shell"
  echo "=========================================================================="
  exit 0
fi

# Build Docker image if requested
if [[ "${ACTION}" == "build" ]] || { [[ "${DRY_RUN}" == false ]] && ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; }; then
  if [[ "${DRY_RUN}" == true ]]; then
    echo "[MetroLidar] Dry-run: would build Docker image ${IMAGE_NAME}."
  else
    echo "=========================================================================="
    echo "[MetroLidar] Building Docker Image: ${IMAGE_NAME}..."
    echo "=========================================================================="
    docker build --build-arg BUILD_DATE="$(date +%s)" -t "${IMAGE_NAME}" -f "${SCRIPT_DIR}/docker/Dockerfile" "${SCRIPT_DIR}"
    if [[ "${ACTION}" == "build" ]]; then
      echo "[MetroLidar] Build completed successfully."
      exit 0
    fi
  fi
fi

# Resolve dataset
RESOLVED_BAG=""
if [[ "${AUTORUN}" == true ]]; then
  echo "[MetroLidar] Auto-run mode enabled (--auto). Scanning for datasets..."
  if ! RESOLVED_BAG="$(find_first_bag)"; then
    echo "[MetroLidar] Error: No datasets found in bags/ or archive/ directory."
    echo "Please copy a ROS 2 bag folder or .db3/.mcap file into '${BAGS_DIR}/'."
    exit 1
  fi
  echo "[MetroLidar] Auto-selected dataset: $(basename "${RESOLVED_BAG}")"
elif [[ -n "${BAG_ARG}" ]]; then
  if ! RESOLVED_BAG="$(resolve_bag "${BAG_ARG}")"; then
    echo "[MetroLidar] Error: Could not resolve dataset '${BAG_ARG}'."
    echo "Looked in: current directory, ${BAGS_DIR}/, and ${ARCHIVE_DIR}/"
    echo ""
    list_available_bags
    exit 1
  fi
  echo "[MetroLidar] Selected dataset: $(basename "${RESOLVED_BAG}")"
fi

# Auto-detect LiDAR topic if not explicitly specified
if [[ -n "${RESOLVED_BAG}" && -z "${LIDAR_TOPIC}" ]]; then
  LIDAR_TOPIC="$(detect_bag_topic "${RESOLVED_BAG}")"
  echo "[MetroLidar] Auto-detected LiDAR topic: ${LIDAR_TOPIC}"
fi
if [[ -z "${LIDAR_TOPIC}" ]]; then
  LIDAR_TOPIC="/lidar_points"
fi

# Configure GPU flags
GPU_FLAGS=""
if [[ "${USE_GPU}" == true ]]; then
  if command -v nvidia-smi >/dev/null 2>&1; then
    GPU_NAME="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n1 || true)"
    echo "[MetroLidar] NVIDIA GPU detected: ${GPU_NAME:-active}"
    GPU_FLAGS="--gpus all"
  else
    echo "[MetroLidar] Notice: NVIDIA GPU not detected on host. Running on CPU."
  fi
fi

# Setup volume mounts
EXTRA_VOLUMES=()
CONTAINER_BAG_PATH=""

if [[ -d "${BAGS_DIR}" ]]; then
  EXTRA_VOLUMES+=("-v" "${BAGS_DIR}:/ros2_ws/bags:ro")
fi
if [[ -d "${ARCHIVE_DIR}" ]]; then
  EXTRA_VOLUMES+=("-v" "${ARCHIVE_DIR}:/ros2_ws/archive:ro")
fi

if [[ -n "${RESOLVED_BAG}" ]]; then
  PLAY_TARGET="${RESOLVED_BAG}"
  if [[ -f "${RESOLVED_BAG}" && -f "$(dirname "${RESOLVED_BAG}")/metadata.yaml" ]]; then
    PLAY_TARGET="$(dirname "${RESOLVED_BAG}")"
  fi

  if [[ "${PLAY_TARGET}" == "${BAGS_DIR}"* ]]; then
    REL_PATH="${PLAY_TARGET#"${BAGS_DIR}/"}"
    CONTAINER_BAG_PATH="/ros2_ws/bags/${REL_PATH}"
  elif [[ -d "${ARCHIVE_DIR}" && "${PLAY_TARGET}" == "${ARCHIVE_DIR}"* ]]; then
    REL_PATH="${PLAY_TARGET#"${ARCHIVE_DIR}/"}"
    CONTAINER_BAG_PATH="/ros2_ws/archive/${REL_PATH}"
  else
    # External path: mount parent folder
    PARENT_DIR="$(cd "$(dirname "${PLAY_TARGET}")" && pwd)"
    BASE_NAME="$(basename "${PLAY_TARGET}")"
    EXTRA_VOLUMES+=("-v" "${PARENT_DIR}:/ros2_ws/external_bags:ro")
    CONTAINER_BAG_PATH="/ros2_ws/external_bags/${BASE_NAME}"
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
    "${EXTRA_VOLUMES[@]}" \
    -v "${SCRIPT_DIR}/docker/cyclonedds.xml:/ros2_ws/cyclonedds.xml:ro" \
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
    "${EXTRA_VOLUMES[@]}" \
    -v "${SCRIPT_DIR}/scripts:/ros2_ws/scripts:ro" \
    -e ROS_DOMAIN_ID=0 \
    "${IMAGE_NAME}" bash -c "python3 /ros2_ws/scripts/verify_system.py"
fi

# Standard run mode
echo "=========================================================================="
echo "Starting MetroLidar2026 Autonomous Stand"
echo "LiDAR Topic:      ${LIDAR_TOPIC}"
echo "Train Speed:      ${TRAIN_SPEED} m/s (~ $(awk "BEGIN {print ${TRAIN_SPEED} * 3.6}") km/h)"
echo "QoS Reliability:  ${QOS_RELIABILITY}"
if [[ -n "${CONTAINER_BAG_PATH}" ]]; then
  echo "Replaying Bag:    ${CONTAINER_BAG_PATH}"
fi
echo "Web Dashboard:    http://localhost:8080 (Enabled: ${ENABLE_WEB_GUI})"
echo "Foxglove Bridge:  ws://localhost:8765   (Enabled: ${ENABLE_FOXGLOVE})"
echo "=========================================================================="

if [[ "${DRY_RUN}" == true ]]; then
  echo "[MetroLidar] Dry-run mode: container would be started with:"
  echo "  Image:          ${IMAGE_NAME}"
  echo "  Container:      ${CONTAINER_NAME}"
  echo "  LiDAR Topic:    ${LIDAR_TOPIC}"
  echo "  Bag Path (in):  ${CONTAINER_BAG_PATH}"
  echo "  Speed:          ${TRAIN_SPEED} m/s"
  echo "  Volumes:        ${EXTRA_VOLUMES[*]}"
  exit 0
fi

# Cleanup previous instance if running
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

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
  "${EXTRA_VOLUMES[@]}" \
  -v "${SCRIPT_DIR}/docker/cyclonedds.xml:/ros2_ws/cyclonedds.xml:ro" \
  -v "${SCRIPT_DIR}/src/metro_web_gui/web:/ros2_ws/install/metro_web_gui/share/metro_web_gui/web:ro" \
  -v "${SCRIPT_DIR}/src/metro_web_gui/web:/ros2_ws/src/metro_web_gui/web:ro" \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///ros2_ws/cyclonedds.xml \
  "${IMAGE_NAME}" \
  ros2 launch metro_obstacle_detector obstacle_detector.launch.py \
    lidar_topic:="${LIDAR_TOPIC}" \
    qos_reliability:="${QOS_RELIABILITY}" \
    train_speed_mps:="${TRAIN_SPEED}" \
    bag_path:="${CONTAINER_BAG_PATH}" \
    launch_web_gui:="${ENABLE_WEB_GUI}" \
    launch_foxglove:="${ENABLE_FOXGLOVE}"
