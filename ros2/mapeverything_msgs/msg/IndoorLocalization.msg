# Indoor-aware Core Location sample with heading and registration quality.
# source and registration_quality are JSON strings.

std_msgs/Header header
string service
float64 latitude
float64 longitude
float64 altitude
float64 ellipsoidal_altitude
float64 horizontal_accuracy
float64 vertical_accuracy
float64 speed
float64 speed_accuracy
float64 course
float64 course_accuracy
bool floor_available
int32 floor_level
bool heading_available
float64 true_heading
float64 magnetic_heading
float64 heading_accuracy
string source
string registration_quality
string timestamp
