# One probe pose on the scan path, in the ARM BASE frame (metres, Z-up).
# Mirrors a waypoint in the pure-Python planner's ScanPath JSON.
geometry_msgs/Pose pose      # probe pose: position (m) + orientation (quaternion)
geometry_msgs/Point target   # surface aim point the probe is facing
float32 incidence_deg        # incidence angle at this waypoint (degrees)
uint32 line_id               # raster line this waypoint belongs to
