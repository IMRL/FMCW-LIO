#!/usr/bin/env bash
set -e

IMAGE_NAME="fmcw_lio:latest"
CONTAINER_NAME="fmcw_lio"

if (($# > 2)); then
    echo "Usage: $0 [workspace_directory] [dataset_directory]"
    exit 1
fi

WORKSPACE_DIR="${1:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)}"
DATASET_DIR="${2:-${HOME}/datasets}"

if [[ ! -d "$WORKSPACE_DIR" ]]; then
    echo "Workspace directory does not exist: $WORKSPACE_DIR"
    echo "Usage: $0 [workspace_directory] [dataset_directory]"
    exit 1
fi

if [[ ! -d "$DATASET_DIR" ]]; then
    echo "Dataset directory does not exist: $DATASET_DIR"
    echo "Usage: $0 [workspace_directory] [dataset_directory]"
    exit 1
fi

# automatically enable NVIDIA GPU support when available
GPU_ARGS=()

if command -v nvidia-smi >/dev/null 2>&1 && \
   nvidia-smi -L >/dev/null 2>&1 && \
   command -v nvidia-container-cli >/dev/null 2>&1; then

    echo "NVIDIA GPU detected. Enabling GPU acceleration."

    GPU_ARGS=(
        --gpus all
        --env NVIDIA_DRIVER_CAPABILITIES=all
    )
else
    echo "No NVIDIA GPU/runtime detected. Using default graphics configuration."
fi

# allow Docker containers to access the X server
xhost +local:docker >/dev/null 2>&1

docker run -it \
    --name "${CONTAINER_NAME}" \
    --privileged \
    "${GPU_ARGS[@]}" \
    --network=host \
    --env "DISPLAY=${DISPLAY}" \
    --env ROS_MASTER_URI=http://localhost:11311 \
    --env ROS_HOSTNAME=localhost \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume "${WORKSPACE_DIR}:/root/fmcw_lio_ws:rw" \
    --volume "${DATASET_DIR}:/datasets:rw" \
    --workdir /root/fmcw_lio_ws \
    "${IMAGE_NAME}"
