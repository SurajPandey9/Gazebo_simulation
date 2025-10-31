# UAV state message

Header header

# Flight mode
string TAKEOFF = "TAKEOFF"
string SEARCH = "SEARCH"
string TRACKING = "TRACKING"
string APPROACH = "APPROACH"
string ENGAGE = "ENGAGE"
string MISSION_COMPLETE = "MISSION_COMPLETE"
string EMERGENCY = "EMERGENCY"

string flight_mode

# Position and orientation
geometry_msgs/Pose pose
geometry_msgs/Twist velocity

# Target information
bool target_detected
int32 target_track_id
float32 distance_to_target

# Mission progress
float32 mission_progress  # 0.0 to 1.0

