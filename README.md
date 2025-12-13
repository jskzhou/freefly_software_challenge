# Offboard Control of Quadrotor for Aerial 3D Printing

This respository implements a simple MAVSDK-based application to control a quadrotor to follow a set of 3D position waypoints. The system is designed with the application of aerial 3D printing in mind.

The code supports:
- generating a trajectory from a text file of 3D waypoints
- smooothing and rate/jerk limiting the trajectory using Ruckig
- offboard control of a quadrotor using MAVSDK-Python to follow the trajectory

To run the code, first clone this repository using:
```
git clone git@github.com:jskzhou/freefly_software_challenge.git
```

Install python dependencies using pip:
```
pip install -r freefly_software_challenge/traj_flyer/requirements.txt
```

The following must be installed separately:
- PX4 Autopilot
- jMAVsim or Gazebo simulator
PX4 SITL setup instructions can be found at https://docs.px4.io/main/en/simulation/

The waypoints are expected to be in a text file named `waypoints.txt` in the following format:
```
north_m east_m down_m
```
with one waypoint per line. Note that the the waypoints in the file are in NED frame, with down being positive. The positions are defined relative to the home position of the drone, with unit in meters.

To generate a trajectory, run:
```
python freefly_software_challenge/traj_flyer/gen_traj_simple.py
```
for a simple trajectory with position and velocity commands, or run:
```
python freefly_software_challenge/traj_flyer/gen_traj_smooth.py
```
for a smoothed and rate/jerk limited trajectory with position, velocity, acceleration commands. The resulting trajectory will be saved to `traj.csv`

To fly the generated trajectory in the simulator, first start the PX4 SITL and simulator (jMAVsim or Gazebo) and wait for `pxh` to show that the drone is ready to be armed. We assume no GCS for this simulation, and can run `param set NAV_DLL_ACT 0` in `pxh` to override the commander health check. Once drone is healthy, run in a separate terminal:
```
python3 ./run_traj.py traj.csv
```
assuming that `run_traj.py` is in the current directory. The drone should take off and follow the trajectory defined in `traj.csv`. The application can be stopped at any time using Ctrl-C, which will safely land the drone before exiting.