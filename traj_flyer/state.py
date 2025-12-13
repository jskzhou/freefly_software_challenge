import asyncio
from typing import Optional

import numpy as np
from mavsdk import System

class DroneState:
    def __init__(self):
        self.pos_ned: Optional[np.ndarray] = None
        self.vel_ned: Optional[np.ndarray] = None
        self.acc_ned: Optional[np.ndarray] = None

        self._last_time: Optional[float] = None # from odometry
        self._prev_time: Optional[float] = None # from previous acceleration update
        self._prev_vel: Optional[np.ndarray] = None

        self._odom_task: Optional[asyncio.Task] = None
        self._posvel_task: Optional[asyncio.Task] = None

    async def start(self, drone: System):
        # Start background tasks to update odometry and position/velocity
        self._odom_task = asyncio.create_task(self._odom_loop(drone))
        self._posvel_task = asyncio.create_task(self._posvel_loop(drone))

    async def stop(self):
        # Cancel background tasks
        if self._odom_task:
            self._odom_task.cancel()
        if self._posvel_task:
            self._posvel_task.cancel()

    async def _odom_loop(self, drone: System):
        async for odom in drone.telemetry.odometry():
            self._last_time = odom.time_usec * 1e-6  # Convert microseconds to seconds
    
    async def _posvel_loop(self, drone: System):
        async for pv in drone.telemetry.position_velocity_ned():
            pos = pv.position
            vel = pv.velocity

            pos_ned = np.array([pos.north_m, pos.east_m, pos.down_m], dtype=float)
            vel_ned = np.array([vel.north_m_s, vel.east_m_s, vel.down_m_s], dtype=float)

            t = self._last_time

            if t is None:
                # Odom time not yet available, only take position and velocity, no acceleration
                self.pos_ned = pos_ned
                self.vel_ned = vel_ned
                self.acc_ned = np.zeros_like(vel_ned)
                continue
            
            self.pos_ned = pos_ned
            self.vel_ned = vel_ned

            # Has valid previous time and previous velocity, compute acceleration
            if self._prev_time is not None and self._prev_vel is not None:
                dt = t - self._prev_time
                if dt > 1e-6:
                    acc_ned = (vel_ned - self._prev_vel) / dt
                    self.acc_ned = acc_ned
                else:
                    self.acc_ned = np.zeros_like(vel_ned)

            # Append current time and velocity as previous for next iteration
            self._prev_time = t
            self._prev_vel = vel_ned