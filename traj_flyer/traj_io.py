from dataclasses import dataclass
import csv
from typing import List, Optional, Tuple

import numpy as np

@dataclass
class Waypoint:
    pos_ned: np.ndarray  # shape (3,)
    vel_ned: np.ndarray  # shape (3,)

def load_trajectory_csv(path: str) -> Tuple[float, List[Waypoint]]:
    dt: Optional[float] = None
    waypoints: List[Waypoint] = []

    # Read all lines first so we can parse metadata cleanly
    with open(path, "r", newline="") as f:
        lines = f.readlines()

    data_lines: List[str] = []
    for line_num, line in enumerate(lines, start=1):
        s = line.strip()
        if not s:
            continue

        if s.startswith("#"):
            meta = s[1:].strip()
            if meta.lower().startswith("dt="):
                val = meta.split("=", 1)[1].strip()
                try:
                    dt = float(val)
                except ValueError as exc:
                    raise ValueError(f"{path}:{line_num}: invalid dt value in '{s}'") from exc
            continue

        data_lines.append(line)

    if dt is None:
        raise ValueError(f"{path}: missing required metadata line '# dt=...'")
    if dt <= 0:
        raise ValueError(f"{path}: dt must be > 0 (got {dt})")

    reader = csv.reader(data_lines)
    for row_num, row in enumerate(reader, start=1):
        if not row:
            continue

        # Skip header row if present
        if row_num == 1 and row[0].strip().lower() in ("north_m", "n", "north"):
            continue

        if len(row) != 6:
            raise ValueError(
                f"{path}:{row_num}: expected 6 columns "
                f"(north_m, east_m, down_m, vn_m_s, ve_m_s, vd_m_s), got {len(row)}"
            )

        try:
            n, e, d, vn, ve, vd = map(float, row)
        except ValueError as exc:
            raise ValueError(f"{path}:{row_num}: non-numeric value in trajectory row: {row}") from exc

        pos = np.array([n, e, d], dtype=float)
        vel = np.array([vn, ve, vd], dtype=float)
        waypoints.append(Waypoint(pos_ned=pos, vel_ned=vel))

    if not waypoints:
        raise ValueError(f"{path}: trajectory CSV is empty (or only contained headers/comments)")

    return dt, waypoints
