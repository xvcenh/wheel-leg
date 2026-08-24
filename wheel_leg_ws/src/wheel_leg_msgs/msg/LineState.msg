# Line detection result for line-following
float32 lateral_error    # Horizontal offset from image center (-1.0 to 1.0, normalized)
float32 angular_error    # Angle of line relative to vertical (radians, positive = tilt right)
float32 confidence       # Detection confidence (0.0 to 1.0)
bool    line_detected    # Whether a line was found in this frame
