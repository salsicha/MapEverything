# Replay fixture extraction

Turns a recorded MapEverything rosbag (`mapeverything_N.db3`, from
`Documents/ROS2Bags/` on the phone) into a replay fixture JSON that the
`StreetScanReplayTests` / `Scan6ReplayTests` loaders parse, so an exact
on-device scan can be replayed and instrumented in unit tests.

```
python3 decode_bag.py /path/to/mapeverything_0.db3   # -> decoded_new.npz next to the script
python3 make_fixture.py [output.json]                # -> fixture JSON (default: MapEverythingTests/Fixtures/StreetScanReplayFrames.json)
python3 roundtrip_check.py                           # sanity: parse, decode a frame, check dims
```

Needs numpy. Notes and traps:

- Bag messages are JSON text (`json.loads(row)['msg']`), binary fields base64.
- The published `depth_anything` clouds are **pre-truncated at the on-device
  per-frame integration cap** — per-frame max camera distance of that cloud
  reconstructs the cap trajectory, but the bag cannot show scene beyond it.
- The published LiDAR cloud is **sparse and range-truncated at 5 m**; the
  device calibrates against the dense 256x192 map, so a replay may drop frames
  the device kept. Treat replay frame drops as fixture artifacts unless the
  device log agrees.
- Raster order of each cloud is verified by re-projection; the relative map is
  rebuilt by inverting the recorded per-frame calibration
  (`metric = 1/(scale*rel + offset)`).
