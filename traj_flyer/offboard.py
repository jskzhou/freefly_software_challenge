import asyncio
import numpy as np

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
from mavsdk.telemetry import LandedState

from traj_io import load_trajectory_csv
from controller import follow_traj


async def fly_traj(
    traj_path: str,
    connection_url: str,
):
    # Load trajectory and dt
    dt, waypoints = load_trajectory_csv(traj_path)
    print(f"Loaded {len(waypoints)} waypoints from {traj_path} (dt={dt})")

    drone = System()
    print(f"Connecting to {connection_url}...")
    await drone.connect(system_address=connection_url)

    # Wait for connection
    async for cs in drone.core.connection_state():
        if cs.is_connected:
            print("Connected to drone.")
            break

    # Wait for basic health
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK, home position OK.")
            break

    print("Arming...")
    await drone.action.arm()

    # Use the first trajectory sample as initial setpoint
    p0 = waypoints[0].pos_ned
    print(f"Setting initial offboard setpoint to first trajectory point: {p0}")
    await drone.offboard.set_position_ned(
        PositionNedYaw(float(p0[0]), float(p0[1]), float(p0[2]), 0.0)
    )

    print("Starting offboard...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        print("Disarming...")
        await drone.action.disarm()
        return

    try:
        print("Following trajectory...")
        await follow_traj(
            drone=drone,
            waypoints=waypoints,
            dt=dt,
        )
    finally:
        print("Stopping offboard...")
        try:
            await drone.offboard.stop()
        except OffboardError as e:
            print(f"Offboard stop failed: {e}")

        print("Landing...")
        await drone.action.land()

        async for ls in drone.telemetry.landed_state():
            if ls == LandedState.ON_GROUND:
                print("Landed.")
                break

        print("Disarming...")
        await drone.action.disarm()