import asyncio
from typing import List

import numpy as np
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw

from traj_io import Waypoint

async def follow_traj(
    drone: System,
    waypoints: List[Waypoint],
    dt: float,
):
    
    if not waypoints:
        print("No waypoints to follow.")
        return

    print(f"Following {len(waypoints)} waypoints")

    for i, wp in enumerate(waypoints):
        p = wp.pos_ned
        v = wp.vel_ned

        pos_msg = PositionNedYaw(
            north_m=float(p[0]),
            east_m=float(p[1]),
            down_m=float(p[2]),
            yaw_deg=0.0,
        )

        vel_msg = VelocityNedYaw(
            north_m_s=float(v[0]),
            east_m_s=float(v[1]),
            down_m_s=float(v[2]),
            yaw_deg=0.0,
        )
        
        await drone.offboard.set_position_velocity_ned(pos_msg, vel_msg)

        # progress print
        if i % 50 == 0 or i == len(waypoints) - 1:
            print(
                f"  Sent waypoint {i+1}/{len(waypoints)}:"
                f" pos=({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}),"
                f" vel=({v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f})"
            )

        await asyncio.sleep(dt)
