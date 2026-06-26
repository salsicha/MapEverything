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
float64 device_pixel_x
float64 device_pixel_y
uint32 tile_width
uint32 tile_height
string pixel_origin
string pixel_units
string format
string mime_type
string source_url
string attribution
string license
string source_policy
bool is_cached
string time
