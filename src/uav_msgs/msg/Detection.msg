# Single object detection message

Header header

# Bounding box in image coordinates
int32 x          # Top-left x coordinate
int32 y          # Top-left y coordinate
int32 width      # Bounding box width
int32 height     # Bounding box height

# Detection metadata
string class_name      # Detected object class (e.g., "tank")
float32 confidence     # Detection confidence [0.0, 1.0]
int32 track_id         # Unique tracking ID (-1 if not tracked)

# 3D position estimate (if available)
geometry_msgs/Point position_3d
bool has_3d_position

