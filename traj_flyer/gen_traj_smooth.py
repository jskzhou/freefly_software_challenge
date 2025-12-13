import argparse
import csv
import numpy as np

from ruckig import Ruckig, InputParameter, OutputParameter, Result

def load_waypoints_xyz(path: str):
    pts = []
    with open(path, "r") as f:
        for line_num, line in enumerate(f, start=1):
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) != 3:
                raise ValueError(f"{path}:{line_num}: expected 3 columns (north east down)")
            n, e, d = map(float, parts)
            pts.append(np.array([n, e, d], dtype=float))

    if len(pts) < 2:
        raise ValueError("File must contain at least two waypoints")
    return pts


def ruckig_segment(p0, v0, a0, p1, v1, a1, v_max, a_max, j_max, dt):
    otg = Ruckig(3, dt)
    inp = InputParameter(3)
    out = OutputParameter(3)

    inp.max_velocity = v_max.tolist()
    inp.max_acceleration = a_max.tolist()
    inp.max_jerk = j_max.tolist()

    inp.current_position = p0.tolist()
    inp.current_velocity = v0.tolist()
    inp.current_acceleration = a0.tolist()

    inp.target_position = p1.tolist()
    inp.target_velocity = v1.tolist()
    inp.target_acceleration = a1.tolist()

    positions = []
    velocities = []

    while True:
        res = otg.update(inp, out)
        if res == Result.Error:
            raise RuntimeError("Ruckig error generating segment")

        positions.append(np.array(out.new_position, dtype=float))
        velocities.append(np.array(out.new_velocity, dtype=float))

        out.pass_to_input(inp)

        if res == Result.Finished:
            break

    return positions, velocities


def main():
    ap = argparse.ArgumentParser(description="Generate jerk-limited trajectory CSV using offline Ruckig")
    ap.add_argument("--waypoints", default="data/waypoints.txt", help="Input waypoint txt (N E D per line)")
    ap.add_argument("--out", default="data/traj.csv", help="Output trajectory CSV")
    ap.add_argument("--dt", type=float, default=0.05, help="Trajectory sample period [s]")
    ap.add_argument("--v_max", type=float, default=1.0, help="Max velocity [m/s] (scalar, all axes)")
    ap.add_argument("--a_max", type=float, default=0.1, help="Max acceleration [m/s^2] (scalar, all axes)")
    ap.add_argument("--j_max", type=float, default=1.0, help="Max jerk [m/s^3] (scalar, all axes)")
    ap.add_argument("--v_takeoff", type=float, default=1.0, help="Optional vertical max speed during takeoff [m/s]")

    args = ap.parse_args()

    if args.dt <= 0:
        raise ValueError("dt must be > 0")
    if args.v_max <= 0 or args.a_max <= 0 or args.j_max <= 0:
        raise ValueError("v_max/a_max/j_max must all be > 0")

    v_max = np.array([args.v_max, args.v_max, args.v_max], dtype=float)
    a_max = np.array([args.a_max, args.a_max, args.a_max], dtype=float)
    j_max = np.array([args.j_max, args.j_max, args.j_max], dtype=float)

    wps = load_waypoints_xyz(args.waypoints)

    # Prepend first point (ground)
    first = wps[0].copy()
    n0, e0, _d0 = first
    wps_full = [np.array([n0, e0, 0.0], dtype=float)] + wps

    all_positions = []
    all_velocities = []

    p_cur = wps_full[0].copy()
    v_cur = np.zeros(3, dtype=float)
    a_cur = np.zeros(3, dtype=float)

    # Allow higher vertical speed during takeoff segment
    v_max_takeoff = v_max.copy()
    v_max_takeoff[2] = max(v_max_takeoff[2], float(args.v_takeoff))

    for i in range(1, len(wps_full)):
        p_tgt = wps_full[i].copy()

        # Try to end each segment at rest
        v_tgt = np.zeros(3, dtype=float)
        a_tgt = np.zeros(3, dtype=float)

        v_use = v_max_takeoff if i == 1 else v_max

        pos_seg, vel_seg = ruckig_segment(
            p_cur, v_cur, a_cur,
            p_tgt, v_tgt, a_tgt,
            v_use, a_max, j_max,
            args.dt,
        )

        # Avoid duplicating the boundary sample between segments
        if i > 1 and len(pos_seg) > 0:
            pos_seg = pos_seg[1:]
            vel_seg = vel_seg[1:]

        all_positions.extend(pos_seg)
        all_velocities.extend(vel_seg)

        p_cur = all_positions[-1].copy()
        v_cur = all_velocities[-1].copy()
        a_cur = np.zeros(3, dtype=float)

    with open(args.out, "w", newline="") as f:
        f.write(f"# dt={args.dt}\n")
        w = csv.writer(f)
        w.writerow(["north_m", "east_m", "down_m", "vn_m_s", "ve_m_s", "vd_m_s"])
        for p, v in zip(all_positions, all_velocities):
            w.writerow([float(p[0]), float(p[1]), float(p[2]),
                        float(v[0]), float(v[1]), float(v[2])])

    print(f"Wrote {len(all_positions)} samples to {args.out} (dt={args.dt})")
    print(f"Limits: v_max={args.v_max}  a_max={args.a_max}  j_max={args.j_max}  v_takeoff={args.v_takeoff}")


if __name__ == "__main__":
    main()