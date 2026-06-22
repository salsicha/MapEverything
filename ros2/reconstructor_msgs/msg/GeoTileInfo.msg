# Georeferencing and source metadata for a satellite imagery tile.
# bounds, device_location, and source_policy are JSON strings.

std_msgs/Header header
string provider
string layer
string kind
string crs
uint32 zoom
uint32 tile_x
uint32 tile_y
string bounds
string device_location
string format
string mime_type
string source_url
string attribution
string license
string source_policy
bool is_cached
string time
