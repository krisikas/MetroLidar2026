# Array of detected obstacles and overall clearance safety status
std_msgs/Header header

float32 train_speed                  # Estimated or received train speed in m/s
float32 min_distance_to_obstacle     # Distance to nearest confirmed obstacle (meters)
float32 time_to_collision            # Time to collision in seconds (-1.0 if clear)
bool emergency_brake_required        # Critical flag for train automated train protection (ATP/АРС)
float32 emergency_braking_distance   # Required braking distance at current speed (meters)

# Overall system alert status
uint8 STATUS_CLEAR = 0               # Path is completely free
uint8 STATUS_NEAR_GAUGE_ADVISORY = 1 # Obstacle detected nearby outside gauge, no collision
uint8 STATUS_TRACK_WARNING = 2       # Potential hazard ahead in clearance envelope
uint8 STATUS_EMERGENCY_BRAKE = 3     # Imminent collision: emergency brake triggered
uint8 alert_status

Obstacle[] obstacles                 # Array of all active obstacle tracks
