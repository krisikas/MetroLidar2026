# 3D Track geometry profile along train route
std_msgs/Header header

float32 rail_head_z                  # Level of Rail Head (УГР) at sensor origin in meters
float32 curvature                    # Path curvature kappa in 1/m (0 = straight track)
float32 grade_slope                  # Longitudinal gradient slope (dz/ds)
float32 roll_cant                    # Track superelevation / cant angle (rad)
float32 track_confidence             # Estimator confidence score [0.0 - 1.0]

geometry_msgs/Point[] waypoints      # 3D centerline waypoints along track
geometry_msgs/Point[] left_rail      # 3D trajectory of left running rail
geometry_msgs/Point[] right_rail     # 3D trajectory of right running rail
