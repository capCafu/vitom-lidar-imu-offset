#!/usr/bin/env bash
# Host entrypoint: build the motiv image if needed, then run the LiDAR<->IMU
# time-offset calibration on a Robin recording folder.
#
#   ./calibrate_offset.sh /path/to/Robin_YYYYMMDD_HHMMSS [options...]
#
# Options are passed through to calibrate_offset.py, e.g.:
#   --all              use the whole bag (default: 60s window from t=5s)
#   --begin 10 --duration 80
#   --no-patch         don't modify sensors_metadata.yaml
#   --lidar-type PANDAR_XT_POINTS
#
# Only Docker is required on the host.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${IKALIBR_MOTIV_IMAGE:-ikalibr-motiv:latest}"

if [ $# -lt 1 ]; then
  echo "usage: $0 /path/to/Robin_folder [--all|--begin N --duration N|--no-patch|--lidar-type T]" >&2
  exit 1
fi

FOLDER="$(realpath "$1")"; shift
if [ ! -d "$FOLDER" ]; then echo "not a folder: $FOLDER" >&2; exit 1; fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[motiv] building $IMAGE (first run only, a few minutes) ..."
  docker build -t "$IMAGE" "$HERE"
fi

# folder mounted read-write so the tool can patch sensors_metadata.yaml
exec docker run --rm -v "$FOLDER":/folder "$IMAGE" /folder "$@"
