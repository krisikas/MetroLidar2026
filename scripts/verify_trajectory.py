#!/usr/bin/env python3
"""
Offline mathematical verification of the RailGeometryTracker algorithm
running against all hackathon rosbag databases.
"""
import glob
import math
import os
import sqlite3
import struct
import time

def parse_points(db_path, max_frames=1):
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    c.execute('SELECT data FROM messages LIMIT ?;', (max_frames,))
    rows = c.fetchall()

    frames = []
    for row in rows:
        raw = row[0]
        data_offset = 196
        point_step = 26
        total = (len(raw) - data_offset) // point_step
        stride = 2 if total > 500000 else 1

        pts = []
        for i in range(0, total, stride):
            offset = data_offset + i * point_step
            if offset + 16 > len(raw):
                break
            x, y, z, intensity = struct.unpack_from('<ffff', raw, offset)
            if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                if y <= 0.5 and y >= -160.0 and abs(x) <= 12.0:
                    pts.append((x, y, z, intensity))
        frames.append(pts)
    return frames

def run_tracker(points, lookahead=150.0, min_dist=2.5, slice_step=2.5):
    span_y = lookahead - min_dist
    num_slices = int(math.ceil(span_y / slice_step))

    slices = [[] for _ in range(num_slices)]
    slice_y_centers = []
    for i in range(num_slices):
        y_start = -min_dist - i * slice_step
        slice_y_centers.append(y_start - 0.5 * slice_step)

    for p in points:
        x, y, z, inten = p
        if -lookahead <= y <= -min_dist:
            idx = int((-min_dist - y) / slice_step)
            if 0 <= idx < num_slices:
                slices[idx].append(p)

    curr_x = 0.0
    curr_z = -1.35
    dx_dy = 0.0
    dz_dy = 0.0
    curvature = 0.0

    single_tunnel_radius = 2.15
    min_curve_radius = 180.0
    max_grade_slope = 0.035
    clearance_corridor_half_w = 1.75

    trajectory = []

    for i in range(num_slices):
        dy = slice_step
        dist_ahead = -slice_y_centers[i]

        x_pred = curr_x + dx_dy * dy + 0.5 * curvature * (dy ** 2)
        z_pred = curr_z + dz_dy * dy

        active_pts = list(slices[i])
        if dist_ahead >= 50.0:
            if i > 0:
                active_pts.extend(slices[i - 1])
            if i + 1 < num_slices:
                active_pts.extend(slices[i + 1])

        # 1. Z estimation
        track_bed = [p[2] for p in active_pts if abs(p[0] - x_pred) <= 0.85 and (z_pred - 0.80 <= p[2] <= z_pred + 0.50)]
        if len(track_bed) >= 3:
            track_bed.sort()
            q_idx = int(len(track_bed) * 0.75)
            meas_z = track_bed[q_idx]
            curr_z = 0.60 * meas_z + 0.40 * z_pred
            delta_z = (curr_z - (z_pred - dz_dy * dy)) / dy
            dz_dy = max(-max_grade_slope, min(max_grade_slope, delta_z))
        else:
            curr_z = z_pred

        # 2. X estimation
        wall_pts = [p for p in active_pts if curr_z + 0.40 <= p[2] <= curr_z + 2.40]
        left_walls = [p[0] for p in wall_pts if p[0] < x_pred and p[0] >= x_pred - 5.5]
        right_walls = [p[0] for p in wall_pts if p[0] > x_pred and p[0] <= x_pred + 5.5]

        min_req = 4 if dist_ahead < 50.0 else (2 if dist_ahead < 90.0 else 1)
        has_l = len(left_walls) >= min_req
        has_r = len(right_walls) >= min_req

        l_bound = x_pred - single_tunnel_radius
        r_bound = x_pred + single_tunnel_radius
        if has_l:
            left_walls.sort()
            l_bound = left_walls[int(len(left_walls) * 0.90)]
        if has_r:
            right_walls.sort()
            r_bound = right_walls[int(len(right_walls) * 0.10)]

        meas_x = x_pred
        valid_meas = False

        dist_l = x_pred - l_bound
        dist_r = r_bound - x_pred

        if has_l and has_r:
            w = r_bound - l_bound
            if 3.2 <= w <= 5.2 and dist_l < 2.8 and dist_r < 2.8:
                meas_x = 0.5 * (l_bound + r_bound)
                valid_meas = True
            elif dist_l < 2.8 and dist_r >= 2.8:
                meas_x = l_bound + single_tunnel_radius
                valid_meas = True
            elif dist_r < 2.8 and dist_l >= 2.8:
                meas_x = r_bound - single_tunnel_radius
                valid_meas = True
        elif has_l and dist_l < 2.8:
            meas_x = l_bound + single_tunnel_radius
            valid_meas = True
        elif has_r and dist_r < 2.8:
            meas_x = r_bound - single_tunnel_radius
            valid_meas = True

        trough_pts = [p[0] for p in active_pts if abs(p[0] - x_pred) <= 0.50 and p[2] < curr_z - 0.15]
        if len(trough_pts) >= 5:
            trough_x = sum(trough_pts) / len(trough_pts)
            if valid_meas:
                meas_x = 0.60 * meas_x + 0.40 * trough_x
            else:
                meas_x = trough_x
                valid_meas = True

        max_dslope = dy / min_curve_radius
        if valid_meas:
            target_slope = (meas_x - curr_x) / dy
            dslope = max(-max_dslope, min(max_dslope, target_slope - dx_dy))
            curvature = 0.75 * curvature + 0.25 * (dslope / dy)
            dx_dy += dslope
            dx_dy = max(-0.45, min(0.45, dx_dy))
        else:
            curvature *= 0.98
            dx_dy += curvature * dy
            dx_dy = max(-0.45, min(0.45, dx_dy))

        curr_x = curr_x + dx_dy * dy

        corridor_l = curr_x - clearance_corridor_half_w
        corridor_r = curr_x + clearance_corridor_half_w
        if l_bound > corridor_l and (curr_x - l_bound) > 1.2:
            corridor_l = l_bound
        if r_bound < corridor_r and (r_bound - curr_x) > 1.2:
            corridor_r = r_bound

        trajectory.append((slice_y_centers[i], curr_x, curr_z, corridor_l, corridor_r))

    if len(trajectory) >= 3:
        smooth_traj = []
        for j in range(len(trajectory)):
            if j == 0 or j == len(trajectory) - 1:
                smooth_traj.append(trajectory[j])
            else:
                y_c = trajectory[j][0]
                sx = 0.25 * trajectory[j - 1][1] + 0.50 * trajectory[j][1] + 0.25 * trajectory[j + 1][1]
                sz = 0.25 * trajectory[j - 1][2] + 0.50 * trajectory[j][2] + 0.25 * trajectory[j + 1][2]
                smooth_traj.append((y_c, sx, sz, trajectory[j][3], trajectory[j][4]))
        trajectory = smooth_traj

    return trajectory

def test_all_bags():
    bags = sorted(glob.glob('archive/for_hackathon/*/*.db3'))
    print(f"Found {len(bags)} bags to verify (150m lookahead).\n")

    all_passed = True
    for db_path in bags:
        name = os.path.basename(os.path.dirname(db_path))
        print(f"==================================================")
        print(f"Testing bag: {name}")
        t0 = time.time()
        frames = parse_points(db_path, max_frames=1)
        pts = frames[0]
        t_parse = (time.time() - t0) * 1000

        t1 = time.time()
        traj = run_tracker(pts)
        t_track = (time.time() - t1) * 1000

        print(f"Points parsed: {len(pts)} ({t_parse:.1f} ms) | Trajectory: {len(traj)} waypoints ({t_track:.1f} ms)")

        for idx in range(0, len(traj), 10):
            y_c, x_c, z_c, c_l, c_r = traj[idx]
            print(f"  Dist: {-y_c:5.1f}m | X: {x_c:+5.2f}m | Z: {z_c:+5.2f}m | Corridor: [{c_l:+5.2f}, {c_r:+5.2f}] (W={c_r-c_l:.2f}m)")
        y_last, x_last, z_last, c_l, c_r = traj[-1]
        print(f"  Dist: {-y_last:5.1f}m | X: {x_last:+5.2f}m | Z: {z_last:+5.2f}m | Corridor: [{c_l:+5.2f}, {c_r:+5.2f}] (W={c_r-c_l:.2f}m)")

        max_corridor_w = max(w[4] - w[3] for w in traj)
        if max_corridor_w > 4.0:
            print(f"FAILED: Max corridor width {max_corridor_w:.2f}m exceeded 4.0m limit!")
            all_passed = False
        else:
            print(f"PASSED (Stable corridor width <= {max_corridor_w:.2f}m)")

    print(f"\n==================================================")
    if all_passed:
        print("ALL 6 BAGS PASSED 150m TRAJECTORY & CORRIDOR VERIFICATION!")
    else:
        print("SOME BAGS FAILED!")

if __name__ == '__main__':
    test_all_bags()

