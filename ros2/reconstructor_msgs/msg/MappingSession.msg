# Session lifecycle and recorder configuration metadata.
# JSON string fields preserve rich app-side dictionaries while keeping this
# message package small enough to build without many support message types.

std_msgs/Header header
string event
string session_id
string state
string recorder_url
string bridge_transport
string[] enabled_streams
string advertised_topics
string radio_channels
string radio_platform_restrictions
string radio_observation_schema
string mesh_snapshot_schema
string stream_payload_metrics
string optional_geo_provider_configurations
string current_wifi_telemetry
string ble_beacon_telemetry
string network_path_diagnostics
string recorder_endpoint_probe
string local_bag_storage
string started_at
string ended_at
string app
string device
PublisherStats publish_queue
string last_error
