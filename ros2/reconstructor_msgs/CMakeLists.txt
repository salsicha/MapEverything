cmake_minimum_required(VERSION 3.8)
project(reconstructor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/MappingSession.msg"
  "msg/GPSMetadata.msg"
  "msg/RadioObservation.msg"
  "msg/GeoTileInfo.msg"
  "msg/GeoRasterTile.msg"
  "msg/IndoorLocalization.msg"
  "msg/MeshSnapshot.msg"
  "msg/PublisherStats.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
