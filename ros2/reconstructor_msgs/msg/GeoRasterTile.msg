# DEM or elevation raster tile payload plus georeferencing metadata.
# bounds, device_location, and source_policy are JSON strings.
# data is a base64-encoded raster payload using the named encoding.

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
string encoding
string data
