# System diagnostics and telemetry for the autonomous metro obstacle detector
std_msgs/Header header

float32 processing_time_ms          # End-to-end processing latency for last frame (target < 30 ms)
float32 fps                         # Effective processing rate in Hz (target 10 Hz)
uint32 input_points                 # Raw point count received from LiDAR
uint32 valid_points                 # Point count after zero-point suppression & ROI filter
uint32 voxel_count                  # Number of active voxels in spatial grid
uint32 candidate_clusters           # Total detected candidate clusters
uint32 confirmed_obstacles          # Total active confirmed obstacles in track clearance
uint32 active_tracks                # Total persistent tracks maintained by Kalman tracker
bool gpu_accelerated                # True if running on NVIDIA CUDA GPU, False if CPU SIMD fallback
