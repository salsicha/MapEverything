# Schema version 1 radio observation emitted on /mapping/radio.

std_msgs/Header header
string session_id
uint32 schema_version
string channel_id
string observation_kind
string source_api
string source_id
string radio_type
string[] tags

bool map_position_available
geometry_msgs/Point map_position
bool geodetic_position_available
float64 latitude
float64 longitude
float64 altitude

string ssid
string bssid
float64 signal_strength_normalized
bool is_secure

string peripheral_id
string local_name
string[] service_uuids
bool is_connectable

string network_status
string[] interface_types
bool is_expensive
bool is_constrained

float64 frequency_hz
float64 rssi_dbm
float64 snr_db
float64 quality

float64 round_trip_time_ms
float64 throughput_bytes_per_second
bool success
string error

string metadata_json
