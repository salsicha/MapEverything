# Calibration paired with /mapping/pointcloud/depth_anything.
# The point cloud carries camera-frame relative Depth Anything coordinates.
# Consumers can rebuild metric depth with:
#   metric_depth_m = scale * relative_depth + offset

std_msgs/Header header
uint32 schema_version
string source
string relative_pointcloud_topic
string overlay_mesh_source
string frame_id
uint32 relative_depth_width
uint32 relative_depth_height
uint32 image_width
uint32 image_height
float64 scale
float64 offset
string equation
string relative_depth_units
string metric_depth_units
string calibration_source
string metadata_json
