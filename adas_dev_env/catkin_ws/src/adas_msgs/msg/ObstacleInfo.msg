# Distance to nearest obstacle ahead, in meters (inf if none detected)
float32 distance

# Relative velocity of obstacle w.r.t. ego vehicle, m/s
# (negative = gap closing, positive = gap opening)
float32 relative_velocity

# Lateral offset of ego vehicle from lane center, in meters
# (positive = vehicle is to the right of center)
float32 lane_offset

# Estimated heading error relative to lane direction, in radians
float32 heading_error

# True if an obstacle was detected within sensor range
bool obstacle_detected
