# Obstacle definition for Autonomous Metro Train GoA4
uint32 id
float32 distance                      # Longitudinal distance ahead in meters
geometry_msgs/Point position          # Center 3D position in LiDAR frame (x, y, z)
geometry_msgs/Vector3 size            # Bounding box dimensions (dx, dy, dz)
geometry_msgs/Vector3 velocity        # Estimated relative velocity vector (vx, vy, vz)
float32 confidence                   # Detection confidence score [0.0 - 1.0]
float32 gauge_penetration            # Signed distance to GOST clearance gauge (negative = inside)

# Threat levels
uint8 THREAT_NONE = 0
uint8 THREAT_CAUTION = 1             # Near gauge / buffer zone
uint8 THREAT_WARNING = 2             # Inside gauge, braking distance allows normal stop
uint8 THREAT_CRITICAL = 3            # Inside gauge, emergency brake required immediately
uint8 threat_level

# Obstacle classifications
uint8 TYPE_UNKNOWN = 0
uint8 TYPE_BULK_BODY = 1             # Large or small 3D volumetric object (2x2m, 0.3x0.3m)
uint8 TYPE_RAIL_SURFACE = 2          # Object lying directly across rails (e.g., 2x0.2m low bar)
uint8 TYPE_SUSPENDED_CABLE = 3       # Thin vertical filament / wire hanging from tunnel roof
uint8 obstacle_type

uint32 point_count                   # Number of raw lidar points supporting this detection
uint32 tracking_frames               # Consecutive frames this object has been tracked
