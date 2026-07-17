# Structured triangle-list mesh snapshot emitted on /mapping/mesh_snapshot.
#
# vertex_data is a base64 rosbridge payload of packed little-endian vertices;
# vertex_encoding and vertex_stride_bytes are self-describing, so consumers
# switch on them rather than on schema_version:
#   float32_xyz_le_base64        stride 12  -> 3x float32 xyz
#   float32_xyz_rgb8_le_base64   stride 15  -> 3x float32 xyz + 3x uint8 rgb
# index_data is packed little-endian uint32 triangle indices (stride 4).
#
# schema_version history:
#   1: initial ARKit triangle-list snapshot
#   2: inverse-depth calibration era
#   3: adds the optional float32_xyz_rgb8_le_base64 colored vertex encoding
#      (emitted for the Depth Anything overlay grid; ARKit meshes stay xyz)

std_msgs/Header header
uint32 schema_version
string snapshot_id
string source
string frame_id
uint32 anchor_count
uint32 vertex_count
uint32 triangle_count
uint32 original_vertex_count
uint32 original_triangle_count
bool is_truncated
uint32 original_payload_bytes
uint32 published_payload_bytes
string compression
string vertex_encoding
uint32 vertex_stride_bytes
uint8[] vertex_data
string index_encoding
uint32 index_stride_bytes
uint8[] index_data
string metadata_json
