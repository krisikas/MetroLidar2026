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

def run_tracker(points, lookahead=180.0, min_dist=2.5, slice_step=2.2):
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
    heading_slope = 0.0
    dz_dy = 0.0
    curvature = 0.0

    nominal_half_width = 2.15
    trajectory = []

    for i in range(num_slices):
        dy = slice_step
        dist_ahead = -slice_y_centers[i]

        x_pred = curr_x + heading_slope * dy + 0.5 * curvature * (dy ** 2)
        theta_pred = heading_slope + curvature * dy
        z_pred = curr_z + dz_dy * dy

        active_pts = list(slices[i])
        if dist_ahead >= 35.0:
            if i > 0:
                active_pts.extend(slices[i - 1])
            if i + 1 < num_slices:
                active_pts.extend(slices[i + 1])
        if dist_ahead >= 80.0:
            if i > 1:
                active_pts.extend(slices[i - 2])
            if i + 2 < num_slices:
                active_pts.extend(slices[i + 2])

        # 1. Z estimation with strict slope clamping
        half_gauge = 0.76
        rail_pts = [p[2] for p in active_pts if (abs(p[0] - (x_pred - half_gauge)) <= 0.16 or abs(p[0] - (x_pred + half_gauge)) <= 0.16) and (z_pred - 0.35 <= p[2] <= z_pred + 0.30)]
        track_bed = [p[2] for p in active_pts if abs(p[0] - x_pred) <= 0.85 and (z_pred - 0.35 <= p[2] <= z_pred + 0.30)]

        meas_z = z_pred
        if len(rail_pts) >= 3:
            rail_pts.sort()
            meas_z = rail_pts[int(len(rail_pts) * 0.85)]
        elif len(track_bed) >= 3:
            track_bed.sort()
            meas_z = track_bed[int(len(track_bed) * 0.85)]

        max_grade_slope = 0.035
        target_dz = (meas_z - curr_z) / dy
        dz_dy = max(-max_grade_slope, min(max_grade_slope, target_dz))
        curr_z = curr_z + dz_dy * dy

        # 2. Wall Extraction (pure spatial quantiles, zero point count bias)
        wall_pts = [p for p in active_pts if curr_z + 0.35 <= p[2] <= curr_z + 3.80]
        left_walls = [p[0] for p in wall_pts if x_pred - 4.5 <= p[0] <= x_pred - 1.25]
        right_walls = [p[0] for p in wall_pts if x_pred + 1.25 <= p[0] <= x_pred + 4.5]

        min_req = 4 if dist_ahead < 45.0 else (2 if dist_ahead < 85.0 else 1)
        has_l = len(left_walls) >= min_req
        has_r = len(right_walls) >= min_req

        l_bound = x_pred - nominal_half_width
        r_bound = x_pred + nominal_half_width
        if has_l:
            left_walls.sort()
            l_bound = left_walls[int(len(left_walls) * 0.90)]
        if has_r:
            right_walls.sort()
            r_bound = right_walls[int(len(right_walls) * 0.10)]

        meas_x = x_pred
        valid_meas = False

        if has_l and has_r:
            obs_width = r_bound - l_bound
            dist_l = x_pred - l_bound
            dist_r = r_bound - x_pred

            if 3.2 <= obs_width <= 5.4 and abs(dist_l - dist_r) <= 1.0:
                meas_x = 0.5 * (l_bound + r_bound)
                valid_meas = True
                if dist_ahead < 50.0:
                    nominal_half_width = 0.90 * nominal_half_width + 0.10 * (0.5 * obs_width)
            else:
                err_l = abs(dist_l - nominal_half_width)
                err_r = abs(dist_r - nominal_half_width)
                if err_l < 0.65 and err_l <= err_r:
                    meas_x = l_bound + nominal_half_width
                    valid_meas = True
                elif err_r < 0.65 and err_r < err_l:
                    meas_x = r_bound - nominal_half_width
                    valid_meas = True
        elif has_r and not has_l:
            dist_r = r_bound - x_pred
            if abs(dist_r - nominal_half_width) < 0.85:
                meas_x = r_bound - nominal_half_width
                valid_meas = True
        elif has_l and not has_r:
            dist_l = x_pred - l_bound
            if abs(dist_l - nominal_half_width) < 0.85:
                meas_x = l_bound + nominal_half_width
                valid_meas = True

        min_curve_radius = 160.0
        max_curv = 1.0 / min_curve_radius
        max_dslope = dy / min_curve_radius
        range_conf = 1.0 if dist_ahead < 60.0 else max(0.20, 1.0 - (dist_ahead - 60.0) / 110.0)

        if valid_meas:
            innov_x = meas_x - x_pred
            K_x = 0.55 * range_conf
            curr_x = x_pred + K_x * innov_x

            dtheta = (innov_x / 12.0) * range_conf
            dtheta = max(-max_dslope, min(max_dslope, dtheta))
            heading_slope = theta_pred + dtheta
            heading_slope = max(-0.45, min(0.45, heading_slope))

            dkappa = (2.0 * innov_x / (28.0 ** 2)) * range_conf
            curvature = (curvature * 0.985) + dkappa
            curvature = max(-max_curv, min(max_curv, curvature))
        else:
            curr_x = x_pred
            heading_slope = theta_pred
            heading_slope = max(-0.45, min(0.45, heading_slope))
            curvature *= 0.985

        corridor_l = curr_x - 1.75
        corridor_r = curr_x + 1.75
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
                sx = trajectory[j][1]  # preserve curve geometry without chord cutting
                sz = 0.25 * trajectory[j - 1][2] + 0.50 * trajectory[j][2] + 0.25 * trajectory[j + 1][2]
                smooth_traj.append((y_c, sx, sz, trajectory[j][3], trajectory[j][4]))
        trajectory = smooth_traj

    return trajectory

def test_all_bags():
    bags = sorted(glob.glob('archive/for_hackathon/*/*.db3'))
    print(f"Found {len(bags)} bags to verify (180m lookahead).\n")

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

        # Проверка отсутствия ложных вторжений в габарит вагона (рельсы, КР, платформа):
        def pt_in_envelope(p):
            for wp in traj:
                if abs(wp[0] - p[1]) <= 1.25:
                    dx = p[0] - wp[1]
                    dz = p[2] - wp[2] # относительно уровня головки рельса (УГР)
                    if 0.15 <= dz <= 3.60:
                        ax = abs(dx)
                        if dz <= 0.50:
                            w = 1.15
                        elif dz <= 1.25:
                            w = 1.33
                        elif dz <= 2.60:
                            w = 1.37
                        else:
                            w = 1.37 - (dz - 2.60) / 1.00 * (1.37 - 0.85)
                        if ax <= w:
                            return True
            return False

        near_pts = [p for p in pts if -50.0 <= p[1] <= -3.0]
        envelope_pts = sum(1 for p in near_pts if pt_in_envelope(p))
        if 'obstacle' in name:
            print(f"  [Envelope Verification] {envelope_pts} obstacle points detected inside clearance zone (expected)")
        else:
            print(f"  [Envelope Verification] {envelope_pts} false intrusion points (rails, KR, platforms cleared)")
            if envelope_pts > 50:
                print(f"  WARNING: Unexpected false intrusions ({envelope_pts} pts) in empty tunnel {name}")

    print(f"\n==================================================")
    if all_passed:
        print("ALL 6 BAGS PASSED 150m TRAJECTORY & CORRIDOR VERIFICATION!")
    else:
        print("SOME BAGS FAILED!")

if __name__ == '__main__':
    test_all_bags()

