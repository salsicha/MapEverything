# MapEverything Dockerized Recorder

One container that runs rosbridge and records the MapEverything topics (including the optional ones) into chunked rosbag2 files, with `mapeverything_msgs` prebuilt.

Quickstart:

1. `cd docker && docker compose up --build`
2. In the MapEverything app, set the **ROS bridge IP** to `ws://<host-ip>:9090` and toggle **ROS** on.
3. Bags land in `docker/bags/mapeverything_<timestamp>/` on the host.
4. Stop with Ctrl-C; the recorder finalizes the bag chunks on shutdown.
5. On macOS/Windows, host networking is unavailable — comment out `network_mode: host` in `docker-compose.yml` and use the provided `ports: "9090:9090"` mapping instead.

Extra flags after `docker compose run recorder <flags>` pass straight through to `tools/run-rosbridge-recorder.py` (e.g. `--record-all`, `--chunk-size-mb 256`, `--storage mcap`).
