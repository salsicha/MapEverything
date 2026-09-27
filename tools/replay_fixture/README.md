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
- Bags recorded before the full-resolution cloud change publish
  `depth_anything` clouds **pre-truncated at the on-device per-frame
  integration cap** — per-frame max camera distance of that cloud
  reconstructs the cap trajectory, but the cloud cannot show scene beyond it.
  Newer builds publish every valid pixel, uncapped.
- Recovering the uncapped depth from an older bag: camera images are
  published only with processed depth frames, so each depth frame has a JPEG
  at the identical timestamp. Running the app's bundled
  `DepthAnythingV2SmallF16` on those JPEGs (through
  `DepthAnythingProcessor.inferRelativeDepth` in a simulator test, from a
  BGRA buffer) reproduced the device's relative maps at correlation >= 0.998
  with an ~identity affine relation, including every beyond-cap pixel. Fit
  that per-frame affine on the overlapping pixels, map the regenerated
  values into device units, and 2x-stride them into the fixture
  (`PanScanReplayFrames.json` was built this way).
- The published LiDAR cloud is **sparse and range-truncated at 5 m**; the
  device calibrates against the dense 256x192 map, so a replay may drop frames
  the device kept. Treat replay frame drops as fixture artifacts unless the
  device log agrees.
- Raster order of each cloud is verified by re-projection; the relative map is
  rebuilt by inverting the recorded per-frame calibration
  (`metric = 1/(scale*rel + offset)`).
