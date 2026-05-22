"""
Trajectory Comparison — Crazyflie
==================================
Compare:
  1. Reference trajectory (commanded)
  2. Qualisys — RIGID BODY CENTER (mean of the 4 markers)
  3. Loco Positioning (loaded from CSV)

Pipeline:
  1) Read QTM TSV + compute the centroid of the 4 markers
  2) Read Loco CSV
  3) Temporal sync via cross-correlation (on the norm ‖xyz‖)
  4) Resampling on a common time grid
  5) Spatial alignment via Kabsch (optimal rotation + translation)
  6) Error computation + plots
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d
from scipy.signal import correlate

# ============================================================
# QUALISYS CONFIG
# ============================================================
QUALISYS_FILE = r"C:\Users\DRONE\Documents\accuracy measurement\test_follow_3.tsv"
QUALISYS_HEADER_LINES = 11
QUALISYS_FREQ = 200
QUALISYS_N_MARKERS = 4
QUALISYS_UNIT_TO_M = 1e-3            # mm -> m

# ============================================================
# LOCO POSITIONING CONFIG (CSV)
# ============================================================
LOCO_FILE = r"C:\Users\DRONE\Documents\accuracy measurement\flight_log_20260507_160105.csv"
LOCO_TIME_COL = "Temps_s"            # time column
LOCO_X_COL = "Drone_X"               # drone XYZ columns only
LOCO_Y_COL = "Drone_Y"               # (Turtle_* and Distance_m columns are ignored)
LOCO_Z_COL = "Drone_Z"
LOCO_TIME_UNIT = "s"                 # "ms" or "s" — here already in seconds
LOCO_UNIT_TO_M = 1.0                 # already in meters
LOCO_CSV_SEP = ","                   # comma

# --- Outlier filtering ---
# Physical bounds of the flight room (in meters). Anything outside = outlier
LOCO_BOUNDS_X = (-5.0, 5.0)
LOCO_BOUNDS_Y = (-5.0, 5.0)
LOCO_BOUNDS_Z = (-0.5, 3.0)          # Z = altitude, negative impossible (or barely)

# Max physical speed of the Crazyflie (m/s). A faster jump = glitch
LOCO_MAX_SPEED = 5.0                 # m/s

# Skip the first N seconds (Kalman filter convergence at startup)
LOCO_SKIP_INITIAL_SEC = 0.0          # set 2.0 or 5.0 if needed

# Target frame: "ref" or "qualisys"
ALIGN_TARGET = "qualisys"

# ============================================================
# ALIGNMENT WINDOW
# ============================================================
# Time segment (in seconds from the start of the common grid)
# used to compute the rotation and translation.
# "auto" -> automatically searches for the segment with the most motion
# Or specify manually: ALIGN_WINDOW = (10.0, 25.0) for example
ALIGN_WINDOW = "auto"                # "auto" or (t_start, t_end) in seconds
ALIGN_WINDOW_DURATION = 15.0         # window duration in auto mode (seconds)

# ============================================================
# ALIGNMENT — MANUAL MODE (optional)
# ============================================================
MANUAL_ALIGN = False
MANUAL_ANGLE_Z_DEG = 180.0
MANUAL_TRANSLATION = [0.0, 0.0, 0.0]

# Manual temporal sync
MANUAL_SYNC = False
MANUAL_TIME_OFFSET = 0.0


# ============================================================
# QUALISYS — read + centroid
# ============================================================
def parse_qualisys_header(filepath, header_lines):
    """
    Parse the QTM header to extract the marker names,
    the frequency, and the number of markers.
    """
    info = {"marker_names": [], "n_markers": 0, "freq": 200}
    with open(filepath, "r") as f:
        for i, line in enumerate(f):
            if i >= header_lines:
                break
            line = line.strip()
            if line.startswith("MARKER_NAMES"):
                # Split on tab, ignore the first element ("MARKER_NAMES")
                parts = line.split("\t")
                info["marker_names"] = [p.strip() for p in parts[1:] if p.strip()]
            elif line.startswith("NO_OF_MARKERS"):
                parts = line.split()
                info["n_markers"] = int(parts[1])
            elif line.startswith("FREQUENCY"):
                parts = line.split()
                info["freq"] = int(parts[1])
    return info


def load_qualisys_centroid(filepath, header_lines, n_markers, unit_scale, freq,
                           entity_prefix="drone"):
    """
    Load a QTM TSV export and compute the drone centroid.

    If the file contains several entities (drone + TurtleBot),
    only the markers whose name starts with `entity_prefix` are used.
    If no name matches (old format without names), all markers
    with n_markers are used (backward-compatible).
    """
    # --- 1) Parse the header for the marker names ---
    header_info = parse_qualisys_header(filepath, header_lines)
    marker_names = header_info["marker_names"]
    total_markers = header_info["n_markers"] or n_markers

    if marker_names:
        print(f"[Qualisys] Markers found in the header: {marker_names}")
        # Drone marker indices
        drone_indices = [i for i, name in enumerate(marker_names)
                         if name.lower().startswith(entity_prefix.lower())]
        other_indices = [i for i in range(len(marker_names))
                         if i not in drone_indices]

        if drone_indices:
            drone_names = [marker_names[i] for i in drone_indices]
            print(f"[Qualisys] -> Selected '{entity_prefix}' markers: "
                  f"{drone_names} (indices {drone_indices})")
            if other_indices:
                other_names = [marker_names[i] for i in other_indices]
                print(f"[Qualisys] -> Ignored markers: {other_names}")
        else:
            print(f"[Qualisys] ! No marker starting with '{entity_prefix}', "
                  f"using the first {n_markers}")
            drone_indices = list(range(min(n_markers, len(marker_names))))
    else:
        print(f"[Qualisys] No MARKER_NAMES in the header, "
              f"using the first {n_markers} markers")
        drone_indices = list(range(n_markers))

    # --- 2) Read the data ---
    df = pd.read_csv(filepath, sep=r"\s+", skiprows=header_lines,
                     header=None, engine="python")
    print(f"[Qualisys] Read: {len(df)} frames, {df.shape[1]} columns")

    data = df.values.astype(float)
    ncols = data.shape[1]

    # Determine whether Frame/Time are present
    n_total = total_markers if total_markers else len(marker_names)
    if ncols == 2 + 3 * n_total:
        t = data[:, 1]
        all_markers_data = data[:, 2:]
    elif ncols == 3 * n_total:
        t = np.arange(len(data)) / freq
        all_markers_data = data
    elif ncols % 3 == 0:
        n_total = ncols // 3
        t = np.arange(len(data)) / freq
        all_markers_data = data
        # Recompute the indices if we detected a different count
        if not marker_names:
            drone_indices = list(range(min(n_markers, n_total)))
    elif (ncols - 2) % 3 == 0:
        n_total = (ncols - 2) // 3
        t = data[:, 1]
        all_markers_data = data[:, 2:]
        if not marker_names:
            drone_indices = list(range(min(n_markers, n_total)))
    else:
        raise ValueError(
            f"[Qualisys] Unable to parse: {ncols} columns.")

    if np.all(t == 0):
        t = np.arange(len(data)) / freq

    # --- 3) Extract only the drone columns ---
    all_markers = all_markers_data.reshape(len(data), n_total, 3)

    # Select the drone markers
    drone_markers = all_markers[:, drone_indices, :]
    n_drone = len(drone_indices)
    print(f"[Qualisys] {n_drone} drone markers extracted out of {n_total} total")

    # --- 4) (0,0,0) = lost marker -> NaN ---
    zero_mask = np.all(drone_markers == 0.0, axis=2)
    n_lost = zero_mask.sum()
    print(f"[Qualisys] Lost drone markers (0,0,0): {n_lost} "
          f"({100*n_lost/(drone_markers.shape[0]*n_drone):.1f} %)")
    drone_markers[zero_mask] = np.nan
    drone_markers *= unit_scale

    # --- 5) Centroid ---
    with np.errstate(invalid="ignore"):
        centroid = np.nanmean(drone_markers, axis=1)

    n_visible = np.sum(~np.isnan(drone_markers[:, :, 0]), axis=1)
    print(f"[Qualisys] Visible drone markers/frame: "
          f"min={n_visible.min()}, max={n_visible.max()}, mean={n_visible.mean():.2f}")

    valid = n_visible >= 2
    if (~valid).sum() > 0:
        print(f"[Qualisys] {(~valid).sum()} frames with <2 drone markers, removed.")

    return t[valid], centroid[valid], drone_markers[valid]


# ============================================================
# LOCO — read CSV
# ============================================================
def filter_outliers(t, xyz, bounds_x, bounds_y, bounds_z, max_speed,
                    skip_initial_sec=0.0):
    """
    Filter LPS outliers:
      1) Skip the first seconds (Kalman convergence)
      2) Physical bounds on X, Y, Z
      3) Max instantaneous speed (the drone cannot teleport)

    Returns cleaned t, xyz.
    """
    n_initial = len(xyz)

    # --- 1) Initial skip ---
    if skip_initial_sec > 0:
        mask_init = (t - t[0]) >= skip_initial_sec
        t, xyz = t[mask_init], xyz[mask_init]
        print(f"[Filter] Skipping first {skip_initial_sec}s: "
              f"{(~mask_init).sum()} samples removed")

    # --- 2) Physical bounds ---
    mask_box = (
        (xyz[:, 0] >= bounds_x[0]) & (xyz[:, 0] <= bounds_x[1]) &
        (xyz[:, 1] >= bounds_y[0]) & (xyz[:, 1] <= bounds_y[1]) &
        (xyz[:, 2] >= bounds_z[0]) & (xyz[:, 2] <= bounds_z[1])
    )
    n_box = (~mask_box).sum()
    if n_box > 0:
        print(f"[Filter] Out of physical bounds: {n_box} samples removed "
              f"({100*n_box/len(xyz):.2f} %)")
    t, xyz = t[mask_box], xyz[mask_box]

    # --- 3) Max speed (iterative filter: one outlier can hide another) ---
    n_speed_total = 0
    for _ in range(5):  # max 5 passes
        if len(xyz) < 2:
            break
        dt = np.diff(t)
        dt[dt <= 0] = 1e-9
        speed = np.linalg.norm(np.diff(xyz, axis=0), axis=1) / dt
        # Flag the 2nd point of a too-fast transition as an outlier
        bad = np.where(speed > max_speed)[0] + 1
        if len(bad) == 0:
            break
        keep = np.ones(len(xyz), dtype=bool)
        keep[bad] = False
        t, xyz = t[keep], xyz[keep]
        n_speed_total += len(bad)
    if n_speed_total > 0:
        print(f"[Filter] Speed > {max_speed} m/s: "
              f"{n_speed_total} samples removed")

    n_final = len(xyz)
    print(f"[Filter] Summary: {n_initial} -> {n_final} samples "
          f"({100*(n_initial-n_final)/n_initial:.2f} % removed)")
    return t, xyz


def load_loco_csv(filepath, time_col, x_col, y_col, z_col,
                  time_unit="ms", unit_scale=1.0, sep=",",
                  bounds_x=None, bounds_y=None, bounds_z=None,
                  max_speed=None, skip_initial_sec=0.0):
    """
    Load a Loco Positioning positions CSV.
    Filters are applied if bounds_* and max_speed are provided.
    """
    df = pd.read_csv(filepath, sep=sep)
    df.columns = df.columns.str.strip()
    print(f"[Loco] Read: {len(df)} samples, columns: {list(df.columns)}")

    for col in [time_col, x_col, y_col, z_col]:
        if col not in df.columns:
            raise KeyError(f"[Loco] Column '{col}' missing from the CSV.")

    t = df[time_col].values.astype(float)
    if time_unit == "ms":
        t = t / 1000.0
    elif time_unit != "s":
        raise ValueError("LOCO_TIME_UNIT must be 'ms' or 's'")
    t = t - t[0]

    xyz = df[[x_col, y_col, z_col]].values.astype(float) * unit_scale

    # NaN
    mask = ~np.isnan(xyz).any(axis=1) & ~np.isnan(t)
    if (~mask).sum() > 0:
        print(f"[Loco] {(~mask).sum()} rows with NaN, removed.")
    t, xyz = t[mask], xyz[mask]

    # Outlier filtering
    if bounds_x is not None and max_speed is not None:
        print("[Loco] Applying anti-outlier filters...")
        t, xyz = filter_outliers(t, xyz, bounds_x, bounds_y, bounds_z,
                                  max_speed, skip_initial_sec)

    print(f"[Loco] -> {len(t)} valid samples, duration {t[-1]-t[0]:.2f} s, "
          f"mean freq {len(t)/(t[-1]-t[0]):.1f} Hz")
    print(f"[Loco] X/Y/Z ranges: "
          f"[{xyz[:,0].min():.2f}, {xyz[:,0].max():.2f}] / "
          f"[{xyz[:,1].min():.2f}, {xyz[:,1].max():.2f}] / "
          f"[{xyz[:,2].min():.2f}, {xyz[:,2].max():.2f}] m")
    return t, xyz


# ============================================================
# TEMPORAL SYNC
# ============================================================
def parse_qualisys_timestamp(filepath, header_lines):
    """Extract the start timestamp from the QTM header (TIME_STAMP line)."""
    from datetime import datetime
    with open(filepath, "r") as f:
        for i, line in enumerate(f):
            if i >= header_lines:
                break
            if line.strip().startswith("TIME_STAMP"):
                parts = line.strip().split("\t")
                # Format: "TIME_STAMP  2026-05-07, 16:01:14.565  ..."
                ts_str = parts[1].strip()  # "2026-05-07, 16:01:14.565"
                try:
                    return datetime.strptime(ts_str, "%Y-%m-%d, %H:%M:%S.%f")
                except ValueError:
                    try:
                        return datetime.strptime(ts_str, "%Y-%m-%d, %H:%M:%S")
                    except ValueError:
                        return None
    return None


def parse_loco_timestamp(filepath):
    """
    Extract the start timestamp from the Loco CSV file name.
    Expected format: flight_log_YYYYMMDD_HHMMSS.csv
    Returns (script start datetime, first timestamp in seconds).
    """
    import os, re
    from datetime import datetime

    basename = os.path.basename(filepath)
    match = re.search(r"(\d{8})_(\d{6})", basename)
    if not match:
        return None, None

    date_str, time_str = match.group(1), match.group(2)
    try:
        script_start = datetime.strptime(f"{date_str}_{time_str}", "%Y%m%d_%H%M%S")
    except ValueError:
        return None, None

    # Read the first timestamp in the CSV
    import pandas as pd
    df = pd.read_csv(filepath, nrows=1)
    first_t = df.iloc[0, 0]  # first column = time

    return script_start, float(first_t)


def estimate_time_offset_from_timestamps(qualisys_file, loco_file, header_lines):
    """
    Compute the time offset from the file timestamps.
    More reliable than cross-correlation since it is content-independent.

    Returns the offset such that t_loco_aligned = t_loco + offset.
    Returns None if the timestamps are not available.
    """
    from datetime import timedelta

    qua_start = parse_qualisys_timestamp(qualisys_file, header_lines)
    loco_start, loco_first_t = parse_loco_timestamp(loco_file)

    if qua_start is None or loco_start is None:
        return None

    # Absolute moment of the first Loco data point
    loco_data_start = loco_start + timedelta(seconds=loco_first_t)

    # Offset = when Loco t=0, where is Qualisys?
    offset = (loco_data_start - qua_start).total_seconds()

    print(f"[Sync] File timestamps:")
    print(f"  Qualisys start : {qua_start.strftime('%H:%M:%S.%f')}")
    print(f"  Loco data start: {loco_data_start.strftime('%H:%M:%S.%f')}")
    print(f"  -> Offset = {offset:+.3f} s")
    return offset


def estimate_time_offset_cross_corr(t1, xyz1, t2, xyz2, dt=0.005):
    """
    Fallback: cross-correlation on the Z axis.
    Used if the timestamps are not available.
    """
    t_overlap = np.arange(max(t1[0], t2[0]), min(t1[-1], t2[-1]), dt)
    if len(t_overlap) < 10:
        print("[Sync] Insufficient overlap, offset = 0")
        return 0.0

    z1 = interp1d(t1, xyz1[:, 2], bounds_error=False, fill_value="extrapolate")(t_overlap)
    z2 = interp1d(t2, xyz2[:, 2], bounds_error=False, fill_value="extrapolate")(t_overlap)
    z1 = (z1 - z1.mean()) / (z1.std() + 1e-9)
    z2 = (z2 - z2.mean()) / (z2.std() + 1e-9)

    corr = correlate(z1, z2, mode="full")
    lags = np.arange(-len(z2) + 1, len(z1)) * dt
    offset = lags[np.argmax(corr)]
    print(f"[Sync] Z cross-correlation: offset = {offset:+.3f} s")
    return offset


def resample_on_grid(t, xyz, t_grid):
    return np.column_stack([
        interp1d(t, xyz[:, i], bounds_error=False, fill_value="extrapolate")(t_grid)
        for i in range(3)
    ])


# ============================================================
# SPATIAL ALIGNMENT
# ============================================================
def kabsch(A, B):
    """Classic Kabsch (free 3D rotation). Kept as a reference."""
    cA, cB = A.mean(0), B.mean(0)
    H = (A - cA).T @ (B - cB)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = cB - R @ cA
    return R, t


def find_best_z_rotation(source_centered, target_centered):
    """
    Search for the rotation angle around Z that minimizes the RMSE.
    Both inputs must be centered (mean = 0).

    1) Coarse search: 1 deg step
    2) Fine search: 0.1 deg step around the best angle
    """
    S, T = source_centered, target_centered
    best_angle, best_rmse = 0, float("inf")

    # Coarse pass
    for deg in range(360):
        theta = np.radians(deg)
        c, s = np.cos(theta), np.sin(theta)
        Rx = c * S[:, 0] - s * S[:, 1]
        Ry = s * S[:, 0] + c * S[:, 1]
        rmse = np.sqrt(np.mean((Rx - T[:, 0])**2 + (Ry - T[:, 1])**2 + (S[:, 2] - T[:, 2])**2))
        if rmse < best_rmse:
            best_rmse, best_angle = rmse, deg

    # Fine pass
    for da in np.arange(-1.0, 1.01, 0.1):
        theta = np.radians(best_angle + da)
        c, s = np.cos(theta), np.sin(theta)
        Rx = c * S[:, 0] - s * S[:, 1]
        Ry = s * S[:, 0] + c * S[:, 1]
        rmse = np.sqrt(np.mean((Rx - T[:, 0])**2 + (Ry - T[:, 1])**2 + (S[:, 2] - T[:, 2])**2))
        if rmse < best_rmse:
            best_rmse, best_angle = rmse, best_angle + da

    return best_angle, best_rmse


def build_z_rotation(angle_deg):
    """Build the rotation matrix around Z."""
    theta = np.radians(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])


def apply_transform(xyz, R, t):
    return (R @ xyz.T).T + t


def find_best_alignment_window(xyz_source, xyz_target, t_grid, duration):
    """
    Find the `duration`-second segment where the two trajectories
    have the most common motion -> best segment to estimate the rotation.

    Score = correlation of the instantaneous speed (norm) between the two signals.
    """
    dt = t_grid[1] - t_grid[0]
    n_win = int(duration / dt)
    if n_win >= len(t_grid):
        return 0, len(t_grid)

    # Instantaneous speed (norm of the displacement between frames)
    v_src = np.linalg.norm(np.diff(xyz_source, axis=0), axis=1)
    v_tgt = np.linalg.norm(np.diff(xyz_target, axis=0), axis=1)

    best_score, best_start = -1, 0
    step = max(1, n_win // 10)  # search step (no need to test every frame)

    for i in range(0, len(v_src) - n_win, step):
        vs = v_src[i:i+n_win]
        vt = v_tgt[i:i+n_win]
        # Score = total motion x speed correlation
        movement = vs.sum() + vt.sum()
        if vs.std() > 1e-6 and vt.std() > 1e-6:
            corr = np.corrcoef(vs, vt)[0, 1]
            if np.isnan(corr):
                corr = 0
        else:
            corr = 0
        score = movement * max(0, corr)
        if score > best_score:
            best_score, best_start = score, i

    t_start_sec = t_grid[best_start] - t_grid[0]
    print(f"  Best segment found: t = {t_start_sec:.1f}s -> "
          f"{t_start_sec+duration:.1f}s (score = {best_score:.1f})")
    return best_start, best_start + n_win


def align_to_target(xyz_source, xyz_target, t_grid=None):
    """
    Robust alignment: Z rotation + translation.

    Alignment window (ALIGN_WINDOW):
      - "auto" -> searches for the segment with the most correlated motion
      - (t_start, t_end) -> fixed segment in seconds

    Computes R and t on the window, then applies them to the WHOLE trajectory.
    """
    N = len(xyz_source)

    # --- Determine the window ---
    if ALIGN_WINDOW == "auto":
        print("  Automatic search for the best segment...")
        i_start, i_end = find_best_alignment_window(
            xyz_source, xyz_target, t_grid, ALIGN_WINDOW_DURATION)
    else:
        t0_win, t1_win = ALIGN_WINDOW
        i_start = int(np.searchsorted(t_grid - t_grid[0], t0_win))
        i_end = int(np.searchsorted(t_grid - t_grid[0], t1_win))
        print(f"  Manual window: t = {t0_win:.1f}s -> {t1_win:.1f}s")

    i_start = max(0, i_start)
    i_end = min(N, i_end)
    n_win = i_end - i_start

    S_win = xyz_source[i_start:i_end]
    T_win = xyz_target[i_start:i_end]
    print(f"  Window: {n_win} points "
          f"(indices {i_start}:{i_end})")

    # --- Search for the best Z angle on the window ---
    cS = S_win.mean(0)
    cT = T_win.mean(0)
    angle, rmse_win = find_best_z_rotation(S_win - cS, T_win - cT)
    R = build_z_rotation(angle)
    t_vec = cT - R @ cS
    print(f"  Z angle found: {angle:.1f} deg, "
          f"window RMSE: {rmse_win*100:.2f} cm")

    # --- Outlier rejection within the window and re-fit ---
    aligned_win = apply_transform(S_win, R, t_vec)
    residuals_win = np.linalg.norm(aligned_win - T_win, axis=1)
    threshold = np.percentile(residuals_win, 85)
    inliers = residuals_win < threshold

    if inliers.sum() > 30:
        S_in, T_in = S_win[inliers], T_win[inliers]
        cS2, cT2 = S_in.mean(0), T_in.mean(0)
        angle2, rmse2 = find_best_z_rotation(S_in - cS2, T_in - cT2)
        R2 = build_z_rotation(angle2)
        t2 = cT2 - R2 @ cS2
        aligned_check = apply_transform(S_in, R2, t2)
        rmse_check = np.sqrt(np.mean(
            np.linalg.norm(aligned_check - T_in, axis=1)**2))
        if rmse_check < rmse_win:
            R, t_vec, angle = R2, t2, angle2
            print(f"  After outlier rejection: Z angle = {angle:.1f} deg, "
                  f"RMSE = {rmse_check*100:.2f} cm, "
                  f"{(~inliers).sum()} outliers rejected")

    # --- Apply to the WHOLE trajectory ---
    aligned_full = apply_transform(xyz_source, R, t_vec)
    residual_full = np.linalg.norm(aligned_full - xyz_target, axis=1)

    print(f"  --- Final result ---")
    print(f"  Z rotation     : {angle:.1f} deg")
    print(f"  Translation    : [{t_vec[0]:+.3f}, {t_vec[1]:+.3f}, {t_vec[2]:+.3f}] m")
    print(f"  RMSE (window)  : {np.sqrt(np.mean(residual_full[i_start:i_end]**2))*100:.2f} cm")
    print(f"  RMSE (total)   : {np.sqrt(np.mean(residual_full**2))*100:.2f} cm")

    return aligned_full, R, t_vec


# ============================================================
# ERRORS
# ============================================================
def compute_errors(measured, reference, label=""):
    diff = measured - reference
    err = np.linalg.norm(diff, axis=1)
    err_ax = np.abs(diff)
    print(f"\n{'='*52}")
    print(f" {label}")
    print(f"{'='*52}")
    print(f"  Mean error  : {err.mean()*100:.2f} cm")
    print(f"  Max error   : {err.max()*100:.2f} cm")
    print(f"  RMSE        : {np.sqrt((err**2).mean())*100:.2f} cm")
    print(f"  Std dev     : {err.std()*100:.2f} cm")
    print(f"  Mean X/Y/Z  : {err_ax[:,0].mean()*100:.2f} / "
          f"{err_ax[:,1].mean()*100:.2f} / {err_ax[:,2].mean()*100:.2f} cm")
    return err


# ============================================================
# PLOTS
# ============================================================
def plot_all(t, ref, qua, loco, err_qua, err_loco):
    has_ref = ref is not None

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")
    if has_ref:
        ax.plot(*ref.T,  "k--", lw=2,   label="Reference")
    ax.plot(*qua.T,  "b-",  lw=1.5, label="Qualisys (centroid)")
    ax.plot(*loco.T, "r-",  lw=1.5, label="Loco Positioning")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("3D trajectories (aligned)"); ax.legend()
    pts_list = [qua, loco] + ([ref] if has_ref else [])
    pts = np.vstack(pts_list)
    rng = (pts.max(0) - pts.min(0)).max() / 2; mid = pts.mean(0)
    for s, m in zip([ax.set_xlim, ax.set_ylim, ax.set_zlim], mid):
        s(m - rng, m + rng)
    plt.tight_layout(); plt.savefig("traj_3d.png", dpi=150)

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for i, lbl in enumerate("XYZ"):
        if has_ref:
            axes[i].plot(t, ref[:, i],  "k--", lw=1.5, label="Ref.")
        axes[i].plot(t, qua[:, i],  "b-",  lw=1,   label="Qualisys")
        axes[i].plot(t, loco[:, i], "r-",  lw=1,   label="Loco")
        axes[i].set_ylabel(f"{lbl} (m)")
        axes[i].legend(loc="upper right", fontsize=8)
        axes[i].grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)"); axes[0].set_title("Position per axis")
    plt.tight_layout(); plt.savefig("traj_axes.png", dpi=150)

    fig, ax = plt.subplots(figsize=(13, 5))
    ax.plot(t, err_loco * 100, "r-", lw=1, label="Loco vs Qualisys")
    if has_ref:
        ax.plot(t, err_qua * 100,  "b-", lw=1, label="Qualisys vs target")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Error (cm)")
    ax.set_title("Euclidean error"); ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout(); plt.savefig("erreurs.png", dpi=150)

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.hist(err_loco * 100, bins=50, alpha=0.6, color="red",  label="Loco vs Qualisys")
    if has_ref:
        ax.hist(err_qua * 100,  bins=50, alpha=0.6, color="blue", label="Qualisys vs target")
    ax.set_xlabel("Error (cm)"); ax.set_ylabel("Frequency")
    ax.set_title("Error distribution"); ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout(); plt.savefig("erreurs_hist.png", dpi=150)
    plt.show()


# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":

    # 1) Qualisys
    print("\n[1] Loading Qualisys + computing centroid")
    t_qua, xyz_qua, _ = load_qualisys_centroid(
        QUALISYS_FILE, QUALISYS_HEADER_LINES,
        QUALISYS_N_MARKERS, QUALISYS_UNIT_TO_M, QUALISYS_FREQ
    )
    print(f"  -> {len(t_qua)} valid frames, duration {t_qua[-1]-t_qua[0]:.2f} s")

    # 2) Loco — CSV loading with outlier filtering
    print("\n[2] Loading Loco CSV")
    t_loco, xyz_loco = load_loco_csv(
        LOCO_FILE, LOCO_TIME_COL, LOCO_X_COL, LOCO_Y_COL, LOCO_Z_COL,
        time_unit=LOCO_TIME_UNIT, unit_scale=LOCO_UNIT_TO_M, sep=LOCO_CSV_SEP,
        bounds_x=LOCO_BOUNDS_X, bounds_y=LOCO_BOUNDS_Y, bounds_z=LOCO_BOUNDS_Z,
        max_speed=LOCO_MAX_SPEED, skip_initial_sec=LOCO_SKIP_INITIAL_SEC,
    )

    # 3) Reference — optional
    t_ref = np.array([])
    xyz_ref = np.empty((0, 3))
    HAS_REF = len(t_ref) > 0

    # ----- Temporal sync -----
    print("\n[3] Temporal synchronization")
    if MANUAL_SYNC:
        print(f"  MANUAL MODE: offset = {MANUAL_TIME_OFFSET} s")
        off_qua = 0.0
        off_loco = MANUAL_TIME_OFFSET
    else:
        # Try first via the file timestamps (more reliable)
        off_ts = estimate_time_offset_from_timestamps(
            QUALISYS_FILE, LOCO_FILE, QUALISYS_HEADER_LINES)
        if off_ts is not None:
            off_qua = 0.0
            off_loco = off_ts
        elif HAS_REF:
            off_qua  = estimate_time_offset_cross_corr(t_ref, xyz_ref, t_qua, xyz_qua)
            off_loco = estimate_time_offset_cross_corr(t_ref, xyz_ref, t_loco, xyz_loco)
        else:
            print("  (timestamps unavailable -> fallback Z cross-correlation)")
            off_qua = 0.0
            off_loco = estimate_time_offset_cross_corr(t_qua, xyz_qua, t_loco, xyz_loco)
    t_qua_s  = t_qua  + off_qua
    t_loco_s = t_loco + off_loco

    # ----- Common grid (coarse, before refinement) -----
    print("\n[4] Resampling")
    if HAS_REF:
        t_start = max(t_ref[0], t_qua_s[0], t_loco_s[0])
        t_end   = min(t_ref[-1], t_qua_s[-1], t_loco_s[-1])
    else:
        t_start = max(t_qua_s[0], t_loco_s[0])
        t_end   = min(t_qua_s[-1], t_loco_s[-1])
    t = np.arange(t_start, t_end, 0.005)
    qua_r  = resample_on_grid(t_qua_s,  xyz_qua,  t)
    loco_r = resample_on_grid(t_loco_s, xyz_loco, t)
    ref_r  = resample_on_grid(t_ref, xyz_ref, t) if HAS_REF else None

    # Filter out NaNs
    ok = ~np.isnan(qua_r).any(axis=1) & ~np.isnan(loco_r).any(axis=1)
    if (~ok).sum() > 0:
        print(f"  {(~ok).sum()} NaN points removed")
        t, qua_r, loco_r = t[ok], qua_r[ok], loco_r[ok]
        if ref_r is not None:
            ref_r = ref_r[ok]
    print(f"  Grid: {len(t)} points over {t[-1]-t[0]:.1f} s")

    # ----- Spatial alignment (1st pass) -----
    print(f"\n[5] Spatial alignment")
    target = qua_r

    if MANUAL_ALIGN:
        print(f"\n-> MANUAL MODE: Z angle = {MANUAL_ANGLE_Z_DEG} deg, "
              f"translation = {MANUAL_TRANSLATION}")
        R = build_z_rotation(MANUAL_ANGLE_Z_DEG)
        t_vec = np.array(MANUAL_TRANSLATION)
        loco_a = apply_transform(loco_r, R, t_vec)
    else:
        print("\n-> Auto alignment (Z rotation, based on the start)")
        loco_a, R_align, t_align = align_to_target(loco_r, target, t_grid=t)

    # ----- Post-alignment temporal refinement -----
    if not MANUAL_SYNC:
        print("\n[5b] Temporal refinement (post-alignment cross-correlation)")
        # Now that the axes are aligned, we can correlate axis by axis
        dt_fine = 0.01
        best_fine_offset = 0.0
        best_fine_corr = -1

        for axis in range(3):
            s1 = qua_r[:, axis]
            s2 = loco_a[:, axis]
            s1 = (s1 - s1.mean()) / (s1.std() + 1e-9)
            s2 = (s2 - s2.mean()) / (s2.std() + 1e-9)

            # Search only within a +/-5s window around 0
            # (the coarse offset is already applied)
            max_shift = int(5.0 / 0.005)  # +/-5s in number of samples
            mid = len(s1) - 1
            corr = correlate(s1, s2, mode="full")
            center = len(s2) - 1
            search = corr[center - max_shift:center + max_shift + 1]
            lags_fine = np.arange(-max_shift, max_shift + 1) * 0.005
            peak = np.argmax(search)
            if search[peak] > best_fine_corr:
                best_fine_corr = search[peak]
                best_fine_offset = lags_fine[peak]

        if abs(best_fine_offset) > 0.01:
            print(f"  Fine correction: {best_fine_offset:+.3f} s")
            off_loco += best_fine_offset
            t_loco_s = t_loco + off_loco

            # Resample again with the corrected offset
            if HAS_REF:
                t_start = max(t_ref[0], t_qua_s[0], t_loco_s[0])
                t_end   = min(t_ref[-1], t_qua_s[-1], t_loco_s[-1])
            else:
                t_start = max(t_qua_s[0], t_loco_s[0])
                t_end   = min(t_qua_s[-1], t_loco_s[-1])
            t = np.arange(t_start, t_end, 0.005)
            qua_r  = resample_on_grid(t_qua_s,  xyz_qua,  t)
            loco_r = resample_on_grid(t_loco_s, xyz_loco, t)
            ok = ~np.isnan(qua_r).any(axis=1) & ~np.isnan(loco_r).any(axis=1)
            if (~ok).sum() > 0:
                t, qua_r, loco_r = t[ok], qua_r[ok], loco_r[ok]
            target = qua_r

            # Re-align spatially with the corrected time
            print("  Re-aligning spatially with the corrected time...")
            if MANUAL_ALIGN:
                loco_a = apply_transform(loco_r, R, t_vec)
            else:
                loco_a, _, _ = align_to_target(loco_r, target, t_grid=t)
        else:
            print(f"  No correction needed (offset < 10 ms)")

    qua_a = qua_r
    ref_a = None
    if HAS_REF:
        print("\n-> Aligning Reference onto Qualisys")
        ref_a, _, _ = align_to_target(ref_r, target, t_grid=t)

    # ----- Errors -----
    print("\n[6] Error metrics")
    err_loco = compute_errors(loco_a, target, "Loco vs Qualisys")
    err_qua  = np.zeros(len(t))
    if HAS_REF:
        err_qua = compute_errors(ref_a, target, "Reference vs Qualisys")

    # ----- Plots -----
    plot_all(t, ref_a, qua_a, loco_a, err_qua, err_loco)
    print("\nOK debug_avant_alignement.png, traj_3d.png, traj_axes.png, erreurs.png, erreurs_hist.png")
