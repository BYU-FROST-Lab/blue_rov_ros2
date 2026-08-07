#!/bin/bash
set -e

# Base ROS 2 bag recording script
# - Sources a parameter file (.sh) for configuration

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARAM_FILE="${SCRIPT_DIR}/record_params.sh"

source "${PARAM_FILE}"

# --------------------------------------------------
# Required variable validation
# --------------------------------------------------
: "${CONTAINER:?Missing CONTAINER in parameter file}"
: "${BAGS_PATH:?Missing BAGS_PATH in parameter file}"
# TOPICS may be set directly or via preset; validated after preset resolution

# Optional defaults (if not set by param file)
STORAGE="${STORAGE:-mcap}"
PRESET="${PRESET:-fastwrite}"
DURATION="${DURATION:-0}"
MAX_CACHE_SIZE="${MAX_CACHE_SIZE:-4096}"

# --------------------------------------------------
# Runtime overrides
# --------------------------------------------------
WAIT_BEFORE_START=0
CUSTOM_BAG_NAME=""

usage() {
  cat <<EOF

Options:
  -p <preset>    Topic preset name (maps to TOPICS_<PRESET> in param file)
  -o <name>      Override bag base name (skip prompt)
  -w <seconds>   Wait before starting recording, -1 to start paused
  -d <seconds>   Duration override
  -c <bytes>     Cache size override
  -t "<topics>" Override topic list (highest priority)
  -h             Show this help and exit
EOF
}
usage() {
  cat <<EOF
Options:
  -o <name>      Override bag base name (skip prompt)
  -w <seconds>   Wait before starting recording, -1 to start paused
  -d <seconds>   Duration override
  -c <bytes>     Cache size override
  -t "<topics>" Override topic list
  -h             Show this help and exit
EOF
}

while getopts ":p:o:w:d:c:t:h" opt; do
  case ${opt} in
    p) PRESET_NAME="$OPTARG" ;;
    o) CUSTOM_BAG_NAME="$OPTARG" ;;
    w) WAIT_BEFORE_START="$OPTARG" ;;
    d) DURATION="$OPTARG" ;;
    c) MAX_CACHE_SIZE="$OPTARG" ;;
    t) TOPICS="$OPTARG" ;;
    h) usage; exit 0 ;;
    \?) echo "Invalid option: -$OPTARG" >&2; usage; exit 1 ;;
  esac
done

# --------------------------------------------------
# Resolve topic preset (if requested)
# --------------------------------------------------
if [ -n "${PRESET_NAME}" ]; then
  PRESET_VAR="TOPICS_${PRESET_NAME^^}"
  if [ -z "${!PRESET_VAR}" ]; then
    echo "Preset '${PRESET_NAME}' not found in parameter file" >&2
    exit 1
  fi
  TOPICS="${!PRESET_VAR}"
fi

: "${TOPICS:?Missing TOPICS after preset resolution}"

# --------------------------------------------------
# Build the exclude argument
# `ros2 bag record -x` takes a single regex, so OR the list together.
# --------------------------------------------------
EXCLUDE_ARG=""
if [ -n "${EXCLUDE_TOPICS// /}" ]; then
  EXCLUDE_REGEX=""
  for t in ${EXCLUDE_TOPICS}; do
    EXCLUDE_REGEX="${EXCLUDE_REGEX:+${EXCLUDE_REGEX}|}${t}"
  done
  EXCLUDE_ARG="-x \"${EXCLUDE_REGEX}\""
fi

# --------------------------------------------------
# Resolve bag name
# --------------------------------------------------
if [ -n "${CUSTOM_BAG_NAME}" ]; then
  folder="${CUSTOM_BAG_NAME}"
else
  echo ""
  echo "IMPORTANT! Name the rosbag with the testing location combined with the test number"
  echo "  ex. 'utah_lake1.0'"
  echo "  - Failed mission: increment second number"
  echo "  - Successful mission: increment first number"
  echo ""
  read -p "Enter a folder name for the rosbag: " folder
fi

TIMESTAMP=$(date +"%Y-%m-%d-%H-%M-%S")
BAG_NAME="${folder}-${TIMESTAMP}"
BAG_DIR="${BAGS_PATH}/${BAG_NAME}"

# --------------------------------------------------
# Summary
# --------------------------------------------------
echo ""
echo "Starting rosbag recording"
echo "  Param file : ${PARAM_FILE}"
echo "  Container  : ${CONTAINER}"
echo "  Bag dir    : ${BAG_DIR}"
echo "  Storage    : ${STORAGE}"
echo "  Preset     : ${PRESET}"
echo "  Duration   : ${DURATION}s"
echo "  Cache      : ${MAX_CACHE_SIZE} bytes"
echo "  Wait       : ${WAIT_BEFORE_START}s"
echo "  Topics     : ${TOPICS}"
echo "  Excluding  : ${EXCLUDE_REGEX:-<none>}"
echo ""

# --------------------------------------------------
# Optional wait
# --------------------------------------------------
if [ "${WAIT_BEFORE_START}" -gt 0 ]; then
  echo "Waiting ${WAIT_BEFORE_START}s before starting..."
  sleep "${WAIT_BEFORE_START}"
fi
if [ "${WAIT_BEFORE_START}" -eq -1 ]; then
  echo "Starting paused. Use Ctrl+C to stop recording."
  read -p "Press Enter to start recording..."
fi

# --------------------------------------------------
# Record rosbag
# --------------------------------------------------
docker exec -it "${CONTAINER}" bash -c "
  source ~/ros2_ws/install/setup.bash && \
  ros2 bag record \
    -s ${STORAGE} \
    --storage-preset-profile ${PRESET} \
    ${MAX_CACHE_SIZE:+--max-cache-size ${MAX_CACHE_SIZE}} \
    ${DURATION:+-d ${DURATION}} \
    -o '${BAG_DIR}' \
    --qos-profile-overrides-path ~/config/qos_record.yaml \
    ${EXCLUDE_ARG} \
    ${TOPICS}
"

# --------------------------------------------------
# Copy configuration snapshot
# --------------------------------------------------
echo "Copying config"
docker exec "${CONTAINER}" bash -c "
  set -e
  cp -r ~/config '${BAG_DIR}/config'
"