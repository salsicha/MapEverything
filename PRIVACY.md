# MapEverything Privacy Policy

Effective date: August 15, 2026

MapEverything does not require an account, display advertising, use analytics,
track users, or send sensor data to the developer.

## Sensor and Mapping Data

MapEverything processes camera images, depth information, LiDAR point clouds,
device pose, location, Bluetooth observations, current-network Wi-Fi
information, and diagnostics on the device.

If the user enables ROS publishing, selected data is sent only to the ROS2
recorder endpoint configured by that user. The developer does not operate,
receive, or have access to that endpoint or its data.

If the user enables Save Local, mapping data is stored on the device in local
bag files. These files leave the device only when the user explicitly shares
them. Users can delete saved sessions from the app.

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
