# Structured triangle-list mesh snapshot emitted on /mapping/mesh_snapshot.

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
