#!/usr/bin/env bash
# Source ROS 2 and the mapeverything_msgs workspace, then run the recorder
# script with all container arguments passed through.
set -e

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /opt/mapeverything_ws/install/setup.bash

# If the caller did not choose an output directory, default to a timestamped
# bag under the /bags volume (the script itself would otherwise default to a
# path relative to the container working directory).
has_output=0
for arg in "$@"; do
    case "$arg" in
        --output|--output=*)
            has_output=1
            break
            ;;
    esac
done

if [ "$has_output" -eq 1 ]; then
    exec python3 /opt/mapeverything/run-rosbridge-recorder.py "$@"
else
    exec python3 /opt/mapeverything/run-rosbridge-recorder.py \
        --output "/bags/mapeverything_$(date +%Y%m%d_%H%M%S)" "$@"
fi
