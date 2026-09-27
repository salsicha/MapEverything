# MapEverything Privacy Policy

Effective date: August 15, 2026

MapEverything does not require an account, display advertising, use analytics,
track users, or send sensor data to the developer.

## Sensor and Mapping Data

MapEverything processes camera images, depth information, LiDAR point clouds,
device pose, motion (IMU) data, location, Bluetooth observations,
current-network Wi-Fi information, and diagnostics on the device.

If the user enables ROS publishing, selected data is sent only to the ROS2
recorder endpoint configured by that user. The developer does not operate,
receive, or have access to that endpoint or its data. The optional session
and radio streams (off by default) include the device name, an app-install
identifier (identifierForVendor), and current-network Wi-Fi details
(SSID/BSSID); recordings that include those streams carry the same values,
so review them before sharing a bag publicly.

If the user enables Resume Scan Area, an ARKit world-map archive (feature
points describing the scanned space) is stored on the device; it is deleted
when the setting is turned off.

If the user enables Save Local, mapping data is stored on the device in local
bag files. The app never uploads these files; they leave the device only when
the user explicitly shares them, or as part of a device or iCloud backup if
the user has backups enabled. Users can delete saved sessions from the app.
Satellite/elevation tiles fetched during scanning are cached on the device
(a coarse, tile-granularity trace of scan locations); the cache is pruned
automatically after 30 days.

## Map and Elevation Providers

To retrieve satellite imagery and elevation data, MapEverything sends
location-derived tile coordinates and ordinary network request information,
including the device's IP address, to public providers such as NASA GIBS,
USGS 3DEP, and Mapzen terrain tiles hosted through AWS. MapEverything does not
add an advertising identifier or user account identifier to these requests.

## Data Retention

The developer does not retain app data. Data stored on the device or on a
user-configured ROS2 recorder remains under the user's control.

## Tracking

MapEverything does not track users across apps or websites and does not share
data for advertising.

## Contact

Questions about this policy may be sent to salsicha@gmail.com or submitted at
https://github.com/salsicha/MapEverything/issues.
