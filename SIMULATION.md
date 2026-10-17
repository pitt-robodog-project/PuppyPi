# PuppyPi Simulation: Mapping and Navigation (ROS 2 Humble, Gazebo Classic)

How to run the PuppyPi in Gazebo, build a map with SLAM, and navigate with Nav2 on a desktop, without the robot. This follows the Hiwonder docs' [SLAM Mapping (ch. 31)](https://docs.hiwonder.com/projects/PuppyPi/en/latest/docs/31.ROS2_SLAM_Mapping_Course.html) and [Autonomous Navigation (ch. 32)](https://docs.hiwonder.com/projects/PuppyPi/en/latest/docs/32.ROS2_Autonomous_Navigation_Course.html) chapters, with the robot hardware replaced by the simulator.

## How the simulation works

The robot is simulated as a **sliding base**: the legs are locked in a standing pose and the whole body glides across the floor. SLAM and Nav2 only use the LiDAR scan, odometry, TF and `/cmd_vel`, so they behave the same as on the real robot. The walking gait is **not** simulated.

| Real robot | Simulation |
|---|---|
| `puppy_control` gait → 8 servos | `gazebo_ros_planar_move` plugin moves the body from `/cmd_vel` |
| LD19 LiDAR driver → `/scan` | Gazebo ray sensor on `lidar_frame` → `/scan` (360°, 450 samples, 10 Hz, 0.12–12 m) |
| rf2o laser odometry + EKF → `/odom` | `planar_move` publishes `/odom` and the `odom → base_footprint` TF |
| Real room | `worlds/puppy_room.world`: a 6 × 4 m room with two boxes |

Files (all in `src/simulations/puppypi_description/`):
- `urdf/puppy_sim.xacro`: includes `puppy.xacro` with `fixed_legs=true` and adds the Gazebo plugins
- `launch/sim.launch.py`: starts Gazebo, `robot_state_publisher` (sim time) and spawns the robot
- `worlds/puppy_room.world`, `rviz/sim.rviz`

Map: `src/slam/maps/sim_room.{yaml,pgm}`, made in this simulation.

The real robot is unaffected: `fixed_legs` defaults to `false`, and the Nav2 launch changes only matter when `use_sim_time:=true`.

## Prerequisites

Ubuntu 22.04 with ROS 2 Humble. On our desktops that's the `robox` distrobox (`distrobox enter robox`).

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard ros-humble-xacro
```

Keep ROS packages at the same release. Nav2 from a newer sync than the rest of ROS fails at startup with `libdiagnostic_updater.so: cannot open shared object file`. Fix with `sudo apt update && sudo apt upgrade`.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select puppypi_description navigation slam
```

Launch files and configs are **copied** into `install/`, so rebuild after editing them.

## Set up every terminal

Each terminal below needs this first:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash        # from the workspace root
export need_compile=True         # Hiwonder launch files crash without it
```

`need_compile=True` makes the launch files use the installed package paths. `False` uses the robot image's `/home/ubuntu/ros2_ws/...` paths, which only exist on the robot.
!!!!! Don't forget this

## Navigation (docs ch. 32)

Three terminals. Start them in this order.

**1. Simulation**
```bash
ros2 launch puppypi_description sim.launch.py
```

**2. Nav2.** This replaces the docs' `ros2 launch navigation navigation.launch.py map:=map_01`, which also starts the robot hardware drivers.
```bash
ros2 launch navigation bringup.launch.py use_sim_time:=true use_teb:=false use_namespace:=false \
    map:=$PWD/src/slam/maps/sim_room.yaml
```
PS: if there is an error did you forget to set use_compile
- `use_teb:=false`: the default TEB controller isn't installed, so this uses DWB, which ships with Nav2.
- `use_namespace:=false` is required when launching `bringup.launch.py` directly. Without it: `launch configuration 'use_namespace' does not exist`.
- `map:=` needs an absolute path, so run this from the workspace root.

Ready when `Managed nodes are active` has appeared **twice** (localization, then navigation).

**3. RViz.** This replaces the docs' `rviz_navigation.launch.py`, which doesn't enable sim time.
```bash
rviz2 -d $(ros2 pkg prefix navigation)/share/navigation/rviz/navigation.rviz --ros-args -p use_sim_time:=true
```
Two display settings need changing each time, or once if you save the config via **File → Save Config As**:
- **Map** display → Topic → **Reliability Policy: Reliable**. Otherwise you get "No map received".
- **RobotModel** display → Description Topic → **Durability Policy: Transient Local**. Otherwise the robot isn't drawn.

### In RViz

1. **2D Pose Estimate** (only if needed). AMCL starts at (0, 0) facing +x, which is where the robot spawns, so after a fresh launch it's already right. If the red laser points don't sit on the black walls, click the robot's real position and drag in the direction it faces. `ros2 topic echo /odom --once --field pose.pose` gives the true pose, which is in map coordinates when the map was built from the spawn point.
2. **2D Goal Pose**: click a free spot and drag to set the final heading. The robot drives there.
3. **Waypoints**: in the **Navigation 2** panel (bottom left, below Pause/Reset; reopen via **Panels → Navigation 2**), click **Waypoint / Nav Through Poses Mode**. Place points with the **Nav2 Goal** toolbar tool, not 2D Goal Pose. Then click **Start Waypoint Following** to stop at each point, or **Start Nav Through Poses** to pass through them.

## Mapping (docs ch. 31)

Only needed to make a new map, for example after changing the world. Don't run SLAM and Nav2 at the same time: both publish `map → odom`.

```bash
# T1: simulation
ros2 launch puppypi_description sim.launch.py

# T2: SLAM. Replaces the docs' slam.launch.py, which also starts the hardware drivers
ros2 launch slam slam_base.launch.py use_sim_time:=true

# T3: RViz with the docs' SLAM config
rviz2 -d $(ros2 pkg prefix slam)/share/slam/rviz/slam.rviz --ros-args -p use_sim_time:=true

# T4: drive slowly (press z a few times); Shift+J / Shift+L strafe
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Cover the whole room, then save (from the workspace root):

```bash
cd src/slam/maps && ros2 run nav2_map_server map_saver_cli -f "sim_room" --free 0.196 \
    --ros-args -p map_subscribe_transient_local:=true
```

`--free 0.196` keeps unknown (grey, 205) pixels as **unknown** when the map is loaded back. With map_saver's default of 0.25 they load as **free**, so Nav2 could plan through unexplored space. The committed `sim_room.yaml` still has `free_thresh: 0.25`. That's harmless in this room, where the only unknown cells are inside the boxes.

Use a name other than `map_01`, which is the real robot's map.

## Limitations

- **No walking.** Legs are locked standing; the body slides. Simulating the gait needs Hiwonder's gait library (`/home/ubuntu/software/puppypi_control` on the robot), `gazebo_ros2_control` and real contact physics.
- **Perfect odometry.** `planar_move` never drifts, so SLAM and AMCL corrections stay near zero. Tuning done here usually needs retuning on the real robot.
- **Slight tilt.** The robot stands about 1.9° nose-up (the front feet reach 5 mm lower), which shows up as a small constant offset in `map → odom`. It doesn't affect 2D mapping or navigation.
- **Global localization is unreliable** with the current AMCL settings (3000 particles, no recovery). Always give an initial pose.
