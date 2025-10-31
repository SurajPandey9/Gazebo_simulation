# Target tracking state message

Header header

# Target identification
int32 track_id
string class_name
bool is_locked        # True if target is being actively tracked

# Target position
geometry_msgs/Pose pose
geometry_msgs/Twist velocity

# Tracking quality
float32 confidence
int32 frames_tracked
int32 frames_lost
bool is_reidentified  # True if target was lost and reacquired

# Visual information
int32 bbox_x
int32 bbox_y
int32 bbox_width
int32 bbox_height

