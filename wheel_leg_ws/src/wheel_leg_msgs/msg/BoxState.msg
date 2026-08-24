# Box detection result for box tracking
float32 bbox_center_x    # Bounding box center X (normalized 0-1)
float32 bbox_center_y    # Bounding box center Y (normalized 0-1)
float32 bbox_width       # Bounding box width (normalized 0-1)
float32 bbox_height      # Bounding box height (normalized 0-1)
float32 distance         # Distance to box in meters (from stereo depth)
float32 confidence       # Detection confidence (0.0 to 1.0)
bool    box_detected     # Whether a box was found in this frame
