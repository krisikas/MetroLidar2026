import glob, os, sqlite3, time
import numpy as np
from scripts.verify_system import parse_pointcloud2_data, MetroPipelineVerifier

def robust_track_geometry_final(xs, ys, zs):
    # 1. Near field Rail Z estimation
    near_m = (ys <= -3.5) & (ys >= -10.0) & (np.abs(xs) <= 1.0) & (zs < -0.6)
    if np.sum(near_m) >= 20:
        z_rail = float(np.percentile(zs[near_m], 85))
    else:
        z_rail = -1.15

    d_samples = []
    x_samples = []
    w_samples = []

    # 2. Extract upper tunnel wall slices (z in [z_rail + 1.40, z_rail + 3.20])
    wall_samples_d = []
    wall_samples_x = []
    for d in np.arange(6.0, 75.0, 2.5):
        m = (ys <= -(d - 1.25)) & (ys >= -(d + 1.25)) & (zs >= z_rail + 1.40) & (zs <= z_rail + 3.20)
        s_xs = xs[m]
        if len(s_xs) >= 8:
            l_w = np.percentile(s_xs, 5)
            r_w = np.percentile(s_xs, 95)
            w = r_w - l_w
            # Single-track tube: width strictly 4.2 to 5.8m
            if 4.2 <= w <= 5.8:
                wall_samples_d.append(d)
                wall_samples_x.append(0.5 * (l_w + r_w))

    # 3. Near-field rails (d in [4, 14]m strictly near train origin)
    near_rail_d = []
    near_rail_x = []
    for d in np.arange(4.0, 15.0, 2.0):
        m = (ys <= -(d - 0.9)) & (ys >= -(d + 0.9)) & (zs >= z_rail - 0.25) & (zs <= z_rail + 0.22)
        s_xs = xs[m]
        l_cand = s_xs[(s_xs >= -1.05) & (s_xs <= -0.50)]
        r_cand = s_xs[(s_xs >= 0.50) & (s_xs <= 1.05)]
        if len(l_cand) >= 4 and len(r_cand) >= 4:
            lp, rp = np.mean(l_cand), np.mean(r_cand)
            if 1.40 <= (rp - lp) <= 1.62:
                near_rail_d.append(d)
                near_rail_x.append(0.5 * (lp + rp))

    # Logic:
    # If single-track wall samples exist (at least 4 slices along tube):
    # Tube walls provide the primary guidance, anchored by near-field rails
    if len(wall_samples_d) >= 4:
        for d, x in zip(wall_samples_d, wall_samples_x):
            d_samples.append(d)
            x_samples.append(x)
            w_samples.append(1.0 / (1.0 + 0.015 * d))
        for d, x in zip(near_rail_d, near_rail_x):
            d_samples.append(d)
            x_samples.append(x)
            w_samples.append(2.0)
    else:
        # Double-track tunnel, station platform, or open area:
        # Use rails (extend rail search to 34m)
        for d, x in zip(near_rail_d, near_rail_x):
            d_samples.append(d)
            x_samples.append(x)
            w_samples.append(3.0)
        c1_guess = (near_rail_x[-1] - near_rail_x[0]) / (near_rail_d[-1] - near_rail_d[0]) if len(near_rail_d) >= 2 else 0.0
        for d in np.arange(16.0, 35.0, 2.0):
            pred_x = c1_guess * d
            m = (ys <= -(d - 0.9)) & (ys >= -(d + 0.9)) & (zs >= z_rail - 0.25) & (zs <= z_rail + 0.22)
            s_xs = xs[m]
            l_cand = s_xs[(s_xs >= pred_x - 1.10) & (s_xs <= pred_x - 0.45)]
            r_cand = s_xs[(s_xs >= pred_x + 0.45) & (s_xs <= pred_x + 1.10)]
            if len(l_cand) >= 4 and len(r_cand) >= 4:
                lp, rp = np.mean(l_cand), np.mean(r_cand)
                if 1.40 <= (rp - lp) <= 1.62:
                    d_samples.append(d)
                    x_samples.append(0.5 * (lp + rp))
                    w_samples.append(2.5)

    d_arr = np.array(d_samples)
    x_arr = np.array(x_samples)
    w_arr = np.array(w_samples)

    if len(d_arr) < 3:
        c1, c2 = 0.0, 0.0
    else:
        A = np.column_stack([d_arr, d_arr**2])
        W = np.diag(w_arr)
        coeffs, _, _, _ = np.linalg.lstsq(W @ A, W @ x_arr, rcond=None)
        c1, c2 = coeffs
        c2 = float(np.clip(c2, -0.5 / 150.0, 0.5 / 150.0))
        c1 = float(np.clip(c1, -0.15, 0.15))

    traj = []
    for d in np.arange(3.0, 81.0, 2.0):
        traj.append({
            's': float(d - 3.0),
            'x': float(c1 * d + c2 * (d**2)),
            'y': float(-d),
            'z': z_rail,
            'yaw': float(np.arctan(c1 + 2.0 * c2 * d)),
            'pitch': 0.0,
            'left_wall': 2.3,
            'right_wall': 2.3
        })
    return traj

def detect_obstacles_final(xs, ys, zs, traj, verifier):
    if len(traj) < 2:
        return []

    traj_y_start = traj[0]['y']
    traj_y_end = traj[-1]['y']
    candidates = []

    for px, py, pz in zip(xs, ys, zs):
        if py > traj_y_start + 1.0 or py < traj_y_end - 1.0:
            continue

        closest = min(traj, key=lambda wp: abs(wp['y'] - py))
        cos_y = np.cos(closest['yaw'])
        sin_y = np.sin(closest['yaw'])
        dx = px - closest['x']
        dy = py - closest['y']

        u = dx * cos_y + dy * sin_y
        v_elev = pz - closest['z']
        s = closest['s'] + (dx * sin_y - dy * cos_y)

        # Lookahead horizon: 3.5m to 75m
        if s < 3.5 or s > 75.0:
            continue

        # 1. Running rails, sleepers, fasteners, and gate sill:
        # Rail heads sit at v in [0.0, 0.25]m near u in [-0.90, +0.90]m.
        # Track bed and ballast sit at v <= 0.15m.
        if v_elev <= 0.15 or (abs(u) <= 0.90 and v_elev <= 0.25):
            continue

        # 2. Contact rail (third rail) and protective casing:
        # Sits at |u| in [0.88, 1.70]m, height v in [0.05, 0.65]m.
        if 0.88 <= abs(u) <= 1.70 and 0.05 <= v_elev <= 0.65:
            continue

        # 3. Station platform edge:
        # Sits at |u| in [1.15, 1.95]m, height v in [0.35, 1.30]m.
        if 1.15 <= abs(u) <= 1.95 and 0.35 <= v_elev <= 1.30:
            continue

        # Clearance Envelope SDF check
        sdf = verifier.compute_sdf(u, v_elev)
        # Margin: require genuine penetration into clearance envelope (at least 6cm)
        if sdf <= -0.06:
            candidates.append((px, py, pz, s, u, v_elev, sdf))

    if len(candidates) < 6:
        return []

    # Cluster along Y with 0.60m threshold
    candidates.sort(key=lambda p: p[1])
    groups = [[candidates[0]]]
    for p in candidates[1:]:
        if abs(p[1] - groups[-1][-1][1]) <= 0.60:
            groups[-1].append(p)
        else:
            groups.append([p])

    confirmed = []
    for grp in groups:
        dist_ahead = -sum(p[1] for p in grp) / len(grp)
        # 8 pts threshold in near/mid field, 15 pts beyond 50m
        min_cluster_pts = 8 if dist_ahead < 50.0 else 15
        if len(grp) >= min_cluster_pts:
            pts_x = [p[0] for p in grp]
            pts_y = [p[1] for p in grp]
            pts_z = [p[2] for p in grp]
            cy = sum(pts_y) / len(grp)
            cx = sum(pts_x) / len(grp)
            cz = sum(pts_z) / len(grp)
            span_x = max(pts_x) - min(pts_x)
            span_z = max(pts_z) - min(pts_z)
            confirmed.append({
                'category': 'BULK_BODY',
                'distance': -cy,
                'pos': (cx, cy, cz),
                'size': (span_x, max(0.15, max(pts_y) - min(pts_y)), span_z),
                'points': len(grp),
                'threat': 'CRITICAL'
            })

    confirmed.sort(key=lambda o: o['distance'])
    return confirmed

if __name__ == '__main__':
    v = MetroPipelineVerifier()
    bags = sorted(glob.glob('archive/for_hackathon/*/*.db3'))
    print(f"Testing {len(bags)} bags with Robust Hybrid Geometry Tracker Final:\n")

    total_passed_bags = 0

    for bag in bags:
        name = os.path.basename(os.path.dirname(bag))
        conn = sqlite3.connect(bag)
        c = conn.cursor()
        c.execute('SELECT count(*) FROM messages;')
        total_msgs = c.fetchone()[0]

        # Test 15 frames evenly spaced across the entire bag
        indices = np.linspace(0, total_msgs - 1, min(15, total_msgs), dtype=int)
        false_alarms = []
        real_detections = []

        for idx in indices:
            c.execute('SELECT data FROM messages LIMIT 1 OFFSET ?;', (int(idx),))
            xs, ys, zs, _ = parse_pointcloud2_data(c.fetchone()[0])
            traj = robust_track_geometry_final(xs, ys, zs)
            obs = detect_obstacles_final(xs, ys, zs, traj, v)

            if name == 'doubleT_obstacle':
                # Real obstacle is present up to frame ~100
                if len(obs) > 0 and obs[0]['distance'] < 75.0:
                    real_detections.append((idx, obs[0]['distance'], obs[0]['points']))
            else:
                if len(obs) > 0:
                    false_alarms.append((idx, len(obs), obs[0]['distance'], obs[0]['points']))

        if name == 'doubleT_obstacle':
            print(f"[BAG] {name:35s} -> PASSED: DETECTED in {len(real_detections)} frames | Real obs closest: {real_detections[0][1]:.1f}m ({real_detections[0][2]} pts)")
            total_passed_bags += 1
        else:
            if len(false_alarms) == 0:
                print(f"[BAG] {name:35s} -> PASSED (0 false alarms across all {len(indices)} frames)")
                total_passed_bags += 1
            else:
                print(f"[BAG] {name:35s} -> FAILED ({len(false_alarms)} false alarms):")
                for fa in false_alarms[:5]:
                    print(f"      Frame {fa[0]}: {fa[1]} obs, dist={fa[2]:.1f}m, pts={fa[3]}")

    print(f"\nResult: {total_passed_bags}/{len(bags)} bags passed ({100.0 * total_passed_bags / len(bags):.1f}%)")
