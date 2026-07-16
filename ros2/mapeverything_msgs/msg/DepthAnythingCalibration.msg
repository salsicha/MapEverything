# Calibration paired with /mapping/pointcloud/depth_anything.
# The point cloud carries camera-frame relative Depth Anything coordinates.
# Depth Anything V2 predicts affine-invariant relative inverse depth
# (disparity), so consumers rebuild metric depth with:
#   metric_depth_m = 1.0 / (scale * relative_depth + offset)
# scale and offset are inverse-depth coefficients in 1/m. The app treats
# reconstructed depths outside (0.1 m, 100 m) as invalid and drops them.
#
# schema_version history:
#   1: metric_depth_m = scale * relative_depth + offset (linear in disparity)
#   2: metric_depth_m = 1.0 / (scale * relative_depth + offset)

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
