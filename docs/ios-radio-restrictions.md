# iOS Radio Telemetry Restrictions

MapEverything only uses public iOS APIs for built-in radio telemetry. That keeps the app deployable on normal devices, but it also means some radio survey data is intentionally unavailable without external hardware or recorder-side integrations.

## Wi-Fi

MapEverything uses `NEHotspotNetwork.fetchCurrent` for current-network Wi-Fi metadata and normalized signal quality. This API reports the network the device is already associated with when iOS grants the required entitlement and permission conditions. It is not a broad Wi-Fi scanner and does not return nearby SSIDs, channels, BSSIDs, AP capabilities, or per-access-point RSSI lists.

Apple also documents `NEHotspotHelper`, but that API is for apps that help users join hotspot networks and requires a special entitlement. MapEverything does not treat it as a general-purpose App Store path for passive AP survey scans.

Operator guidance:

- Treat `/mapping/radio` Wi-Fi observations as current associated network quality only.
- For AP survey heatmaps, use an external Wi-Fi scanner, router/controller API, or companion ROS2 node.
- Fuse external Wi-Fi observations by timestamp and, when available, MapEverything pose or GPS.

## Cellular

Public iOS APIs can report some cellular service and network path state, but they do not provide a reliable raw cellular RF stream such as RSSI, RSRP, RSRQ, or SINR for field mapping. MapEverything therefore does not publish built-in cellular signal strength measurements.

Operator guidance:

- Treat Network.framework cellular observations as path/interface state, not RF power.
- For cellular RF surveys, record telemetry from a modem, router, external scanner, SDR, or companion ROS2 node.
- Publish those external measurements as `external_adapter` radio observations with frequency, RSSI, SNR, quality, and adapter metadata.

## BLE

CoreBluetooth can report BLE advertisement RSSI for discovered peripherals. MapEverything scopes scans to configured service UUIDs, peripheral UUIDs, or local-name prefixes. Background scanning behavior is limited by iOS policy, so field validation should be done on physical devices in the foreground.

## Public API References

- [NEHotspotNetwork](https://developer.apple.com/documentation/networkextension/nehotspotnetwork)
- [NEHotspotNetwork.fetchCurrent](https://developer.apple.com/documentation/networkextension/nehotspotnetwork/fetchcurrent(completionhandler:))
- [NEHotspotHelper](https://developer.apple.com/documentation/networkextension/nehotspothelper)
- [Core Telephony](https://developer.apple.com/documentation/coretelephony)
- [NWPathMonitor](https://developer.apple.com/documentation/network/nwpathmonitor)
- [CBCentralManager](https://developer.apple.com/documentation/corebluetooth/cbcentralmanager)
