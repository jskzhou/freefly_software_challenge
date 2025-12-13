# Offboard Control of Quadrotor for Aerial 3D Printing

This repository implements a simple MAVSDK-based application to control a quadrotor to follow a set of 3D position waypoints. The system is designed with the application of aerial 3D printing in mind.

The code supports:
- generating a trajectory from a text file of 3D waypoints
- smoothing and rate/jerk limiting the trajectory using Ruckig
- offboard control of a quadrotor using MAVSDK-Python to follow the trajectory

To run the code, first clone this repository. Install python dependencies using pip:
```
pip install -r freefly_software_challenge/traj_flyer/requirements.txt
```

The following must be installed separately:
- PX4 Autopilot
- jMAVSim or Gazebo simulator

PX4 SITL setup instructions can be found at:
https://docs.px4.io/main/en/simulation/

---

## Waypoint Format
Waypoints are provided in a text file named `waypoints.txt` with the following format:
```
north_m east_m down_m
```
One waypoint is specified per line. Waypoints are expressed in the local NED frame, with positive down. Positions are defined relative to the vehicle home position and use meters as units.

---

## Trajectory Generation

To generate a simple trajectory with position and velocity setpoints, run:
```
python3 traj_flyer/gen_traj_simple.py
```
To generate a smoothed, rate- and jerk-limited trajectory using Ruckig, run:
```
python3 traj_flyer/gen_traj_smooth.py
```
Both scripts accept optional command-line arguments (e.g., timestep, velocity limits). If not specified, defaults used to generate results are used. Append `-h` to either script to see usage information. The generated trajectory is saved to `traj.csv` and includes metadata such as the trajectory timestep.

---

## Running the Trajectory in Simulation

Start the PX4 SITL and simulator (jMAVSim or Gazebo) and wait for `pxh` to indicate the vehicle is ready to arm. This setup assumes no GCS is connected; the commander health check can be overridden by running `param set NAV_DLL_ACT 0` in the `pxh` console. Once the vehicle is healthy, run the trajectory follower from a separate terminal:
```
python3 traj_flyer/run_traj.py data/traj.csv
```
The vehicle will take off and follow the trajectory defined in `traj.csv`. The application can be stopped at any time using Ctrl-C, which will safely land the vehicle before exiting.

The resulting videos are too large, and can be viewed at https://drive.google.com/drive/folders/1kACAoR3D9zd-AgzvUJqLdyq_KxkIufGc?usp=sharing