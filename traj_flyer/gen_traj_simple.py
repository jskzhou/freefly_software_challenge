import argparse
import csv
import math
import numpy as np


def load_waypoints_xyz(path: str):
    points = []
    with open(path, "r") as f:
        for line_num, line in enumerate(f, start=1):
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) != 3:
                raise ValueError(f"{path}:{line_num}: expected 3 columns (north east down)")
            n, e, d = map(float, parts)
            points.append(np.array([n, e, d], dtype=float))

    if len(points) < 2:
        raise ValueError("File must contain at least two waypoints")
    return points


def generate_segment(p0: np.ndarray, p1: np.ndarray, speed: float, ds: float):
    delta = p1 - p0
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        return [], []

    direction = delta / length
    velocity = direction * speed

    n_steps = max(2, int(math.ceil(length / ds)))

    positions = []
    velocities = []
    for k in range(n_steps):
        alpha = k / n_steps
        pos = (1.0 - alpha) * p0 + alpha * p1
        positions.append(pos)
        velocities.append(velocity)

    return positions, velocities


def generate_vertical_takeoff(north: float, east: float, down_target: float, v_takeoff: float, ds: float):
    down_start = 0.0
    dz = down_target - down_start
    if abs(dz) < 1e-6:
        return [], []

    dist = abs(dz)
    n_steps = max(2, int(math.ceil(dist / ds)))

    vd = -v_takeoff if down_target < down_start else v_takeoff

    positions = []
    velocities = []
    for k in range(n_steps):
        alpha = k / n_steps
        down = (1.0 - alpha) * down_start + alpha * down_target
        pos = np.array([north, east, down], dtype=float)
        vel = np.array([0.0, 0.0, vd], dtype=float)
        positions.append(pos)
        velocities.append(vel)

    return positions, velocities


def main():
    ap = argparse.ArgumentParser(description="Generate linear trajectory CSV from waypoint text file")
    ap.add_argument("--waypoints", default="data/waypoints.txt", help="Input waypoint txt (N E D per line)")
    ap.add_argument("--out", default="data/traj.csv", help="Output trajectory CSV")
    ap.add_argument("--dt", type=float, default=0.05, help="Trajectory sample period [s]")
    ap.add_argument("--v_max", type=float, default=0.25, help="Nominal segment speed [m/s]")
    ap.add_argument("--v_takeoff", type=float, default=1.0, help="Vertical takeoff speed [m/s]")

    args = ap.parse_args()

    if args.dt <= 0:
        raise ValueError("dt must be > 0")
    if args.v_max <= 0:
        raise ValueError("v_max must be > 0")

    ds = args.v_max * args.dt
    waypoints = load_waypoints_xyz(args.waypoints)

    all_positions = []
    all_velocities = []

    # Prepend takeoff sequence
    first = waypoints[0]
    n0, e0, d0 = float(first[0]), float(first[1]), float(first[2])

    pos_to, vel_to = generate_vertical_takeoff(n0, e0, d0, args.v_takeoff, ds)
    all_positions.extend(pos_to)
    all_velocities.extend(vel_to)

    # Follow waypoint segments
    for i in range(len(waypoints) - 1):
        p0 = waypoints[i]
        p1 = waypoints[i + 1]
        pos_seg, vel_seg = generate_segment(p0, p1, args.v_max, ds)
        all_positions.extend(pos_seg)
        all_velocities.extend(vel_seg)

    with open(args.out, "w", newline="") as f:
        f.write(f"# dt={args.dt}\n")
        w = csv.writer(f)
        w.writerow(["north_m", "east_m", "down_m", "vn_m_s", "ve_m_s", "vd_m_s"])
        for p, v in zip(all_positions, all_velocities):
            w.writerow([float(p[0]), float(p[1]), float(p[2]), float(v[0]), float(v[1]), float(v[2])])

    print(f"Wrote {len(all_positions)} samples to {args.out} (dt={args.dt}, ds={ds})")


if __name__ == "__main__":
    main()