# MapEverything recorder container: rosbridge_server + chunked rosbag2 recording
# with the mapeverything_msgs custom message package prebuilt.
#
# Build from the repository root so ros2/ and tools/ are in the build context:
#   docker build -f docker/Dockerfile -t mapeverything-recorder .
# or simply `docker compose up --build` from the docker/ directory.
FROM ros:jazzy-ros-base

# rosbridge, rosbag2 with its default storage plugins (sqlite3 is the recorder
# script's default --storage), the mcap storage plugin, and YAML support.
# build-essential/colcon are already in ros-base images but are listed
# explicitly so the build keeps working if the base image slims down.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rosbridge-suite \
        ros-jazzy-rosbag2 \
        ros-jazzy-rosbag2-storage-sqlite3 \
        ros-jazzy-rosbag2-storage-mcap \
        python3-yaml \
        python3-colcon-common-extensions \
        build-essential \
    && rm -rf /var/lib/apt/lists/*

# Build the MapEverything custom message package into its own workspace.
ENV MAPEVERYTHING_WS=/opt/mapeverything_ws
COPY ros2/mapeverything_msgs ${MAPEVERYTHING_WS}/src/mapeverything_msgs
RUN . /opt/ros/jazzy/setup.sh \
    && cd ${MAPEVERYTHING_WS} \
    && colcon build --packages-select mapeverything_msgs

# The recorder orchestration script (starts rosbridge + ros2 bag record).
COPY tools/run-rosbridge-recorder.py /opt/mapeverything/run-rosbridge-recorder.py
COPY docker/entrypoint.sh /opt/mapeverything/entrypoint.sh
RUN chmod +x /opt/mapeverything/entrypoint.sh

# Bags default under /bags (mounted as a volume by docker-compose.yml).
VOLUME ["/bags"]
EXPOSE 9090

# The entrypoint sources ROS + the workspace, then runs the recorder script
# with any arguments passed through.
ENTRYPOINT ["/opt/mapeverything/entrypoint.sh"]
CMD []
