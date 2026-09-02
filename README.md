# tb3_multi_robot — follow branch

This branch runs a two-robot ROS 2 Jazzy / Gazebo Harmonic scenario in which
`tb1` is the leader and `tb3` receives dynamically generated trailing goals.

## Quick start with Docker

Run these commands from the repository root. The image is built from the local
checkout, so uncommitted source changes are included.

Headless (the easiest smoke-test path):

```bash
TB3_GUI=false TB3_RVIZ=false \
  docker compose -f docker/docker-compose.yaml up --build
```

With Gazebo and RViz windows on a Linux X11 desktop:

```bash
xhost +local:docker
docker compose -f docker/docker-compose.yaml up --build
```

Revoke the temporary X11 permission when finished:

```bash
xhost -local:docker
```

Compose waits for live odometry from both spawned robots before starting Nav2.
It also defaults to ROS domain 42 so the Jazzy containers do not collide with
another ROS graph on the host. Override that domain when needed:

```bash
TB3_ROS_DOMAIN_ID=73 docker compose -f docker/docker-compose.yaml up
```

Gazebo retains its 1 kHz / 1 ms physics integration for contact and motion
accuracy. The native clock is available as `/clock_raw`; `/clock` caps
exact-sample forwarding at 250 Hz by default, preventing every Nav2 node from
processing every physics tick. Override the fan-out rate without changing the
physics rate:

```bash
TB3_CLOCK_RATE=500 docker compose -f docker/docker-compose.yaml up
```

The Docker healthcheck is backed by a persistent odometry-freshness monitor.
It detects stale robot simulation continuously without repeatedly starting ROS
CLI processes and new DDS participants.

Both RViz views retain all configured displays while rendering at 10 FPS. On
hosts without a container-accessible GPU, Mesa is limited to two render threads
per RViz process. Override that limit when more rendering capacity is useful:

```bash
TB3_RVIZ_RENDER_THREADS=4 docker compose -f docker/docker-compose.yaml up
```

Useful commands:

```bash
# Start in the background
TB3_GUI=false TB3_RVIZ=false \
  docker compose -f docker/docker-compose.yaml up -d

# Inspect service state and logs
docker compose -f docker/docker-compose.yaml ps
docker compose -f docker/docker-compose.yaml logs -f nav

# Open a ROS-aware shell in the navigation container
docker compose -f docker/docker-compose.yaml exec nav bash

# Stop and remove the scenario containers
docker compose -f docker/docker-compose.yaml down
```

The default AMCL poses match the spawn locations in `config/robots.yaml`:
`tb1=(-1.5, -0.5)` and `tb3=(1.5, -0.5)`. If those spawn positions change,
update the `amcl.ros__parameters.initial_pose` values in the two burger Nav2
parameter files as well.

## Native Jazzy workspace

From the root of a ROS 2 Jazzy workspace containing this repository under
`src/tb3_multi_robot`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/tb3_multi_robot --ignore-src --rosdistro jazzy -r -y
colcon build --packages-select tb3_multi_robot --symlink-install
source install/setup.bash
ros2 launch tb3_multi_robot follow_sim.launch.py
```

The combined launch accepts `gui:=false`, `rviz:=false`, `clock_rate:=500`,
`follow_distance:=0.8`, and `publish_rate:=1.5` overrides. You can also launch
the world and navigation separately with `tb3_world.launch.py` and
`follow_tb3.launch.py`.

## Driving the leader

Send a Nav2 goal to `tb1` from its RViz window, or from the navigation
container:

```bash
ros2 action send_goal /tb1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: -0.5, y: -0.5}, orientation: {w: 1.0}}}}'
```

Once AMCL observes leader motion, `tb3_follow_tb1` sends trailing
`NavigateToPose` goals to `/tb3/navigate_to_pose`.

<details>
<summary>Legacy branch and upstream documentation</summary>

# Modifications

## TB3 Follow Mode — tb3 Follows tb1

This section describes the **follow scenario**: tb1 is driven manually via RViz waypoints while tb3 autonomously trails 0.5 m behind it using Nav2's Follow Dynamic Point behavior tree.

### New Files

| File | Location | Purpose |
|---|---|---|
| `tb3_follow_tb1.py` | `multi_robot_scripts/` | Publishes a trailing goal pose to `/tb3/goal_pose` based on tb1's odometry |
| `follow_tb3.launch.py` | `launch/` | Launches Nav2 for both robots (separate params) + the follow node |
| `burger_nav2_params_tb3.yaml` | `params/` | tb3-specific Nav2 params: FollowDynamicPoint BT, relaxed goal tolerances, wider costmap |

`setup.py` must have the following entry added to `console_scripts`:

```python
'tb3_follow_tb1 = multi_robot_scripts.tb3_follow_tb1:main',
```

***

### Docker Setup

The follow scenario runs entirely inside Docker. Two containers are used:

| Container | Service name | Runs |
|---|---|---|
| `tb3_gazebo` | `gazebo` | Gazebo world + robot spawning |
| `tb3_nav` | `nav` | Nav2 (tb1 + tb3) + follow node + RViz (x2) |

The `docker-compose.yaml` and updated `Dockerfile` / `entrypoint.sh` live in `docker/`.

#### One-Time: Build the Image

Only needed once, or after changes to the `Dockerfile` itself:

```bash
cd docker
docker compose build
```

To force a fresh pull of the latest `follow` branch (e.g. after pushing new commits):

```bash
docker compose build --no-cache
```

> **Note:** The image clones from the `follow` branch of this repo at build time:
> ```dockerfile
> RUN git clone https://github.com/gglaspell/tb3_multi_robot.git src/tb3_multi_robot -b follow
> ```
> Any code changes pushed to `follow` require a rebuild to take effect inside the container.

***

#### Run Once Per Session: Gazebo World

Start Gazebo first and leave it running. This spawns the world and both robots:

```bash
# From the docker/ directory
docker compose up gazebo
```

Wait until you see Gazebo fully loaded before starting Nav2.

***

#### Run Once Per Session: Nav2 + Follow Node

In a second terminal, start Nav2 and the follow node:

```bash
docker compose up nav
```

This launches:
- Nav2 for **tb1** using `burger_nav2_params.yaml` (standard `NavigateToPose`)
- Nav2 for **tb3** using `burger_nav2_params_tb3.yaml` (FollowDynamicPoint BT)
- Two RViz windows (one per robot)
- `tb3_follow_tb1` node publishing the dynamic trailing goal

***

#### Development Workflow: Updating Code

When you change Python scripts or launch files and want to test without a full rebuild:

```bash
# 1. Push your changes to the follow branch
git push origin follow

# 2. Rebuild to pull the latest commit
docker compose build --no-cache

# 3. Restart both services
docker compose down
docker compose up gazebo   # terminal 1
docker compose up nav      # terminal 2
```

If you only changed parameters (`.yaml` files), a rebuild is still required because params are installed into the image at build time via `colcon build`.

***

#### Useful Docker Commands

```bash
# Shell into the running nav container (e.g. to run ros2 topic echo)
docker compose exec nav bash

# Override follow_distance at run time (default 0.5 m)
docker compose run nav ros2 launch tb3_multi_robot follow_tb3.launch.py follow_distance:=0.8

# Stop all containers
docker compose down

# View logs from the follow node only
docker compose logs nav | grep tb3_follow_tb1
```

***

### Running the Follow Scenario Step by Step

#### Step 1 — Set tb1's Initial Pose

In the **tb1 RViz window**:

1. Click **2D Pose Estimate** in the toolbar
2. Click and drag on the map to place tb1 at its physical starting position, matching where it was spawned in Gazebo (default: `x=-1.5, y=-0.5`)
3. Confirm AMCL has localised: the particle cloud should converge around the robot

#### Step 2 — Set tb3's Initial Pose

In the **tb3 RViz window**:

1. Click **2D Pose Estimate**
2. Place tb3 at its spawned position (default: `x=1.5, y=-0.5`)
3. Confirm AMCL localisation in the same way

> tb3 does not need a navigation goal sent manually — the `tb3_follow_tb1` node will begin publishing goals to `/tb3/goal_pose` automatically once tb1 starts moving.

#### Step 3 — Verify the Follow Node is Publishing

Before driving tb1, confirm the follow node is active:

```bash
docker compose exec nav bash
ros2 topic echo /tb3/goal_pose
```

The topic will be silent until tb1 moves more than 0.1 m (the deadband threshold). This is expected.

#### Step 4 — Drive tb1 with Waypoints

In the **tb1 RViz window**:

1. Click **Nav2 Goal** (or **Navigation2 → Send Goal**) in the toolbar
2. Click and drag on the map to set a goal pose for tb1
3. tb1 will navigate to the goal using standard Nav2

As tb1 moves, `tb3_follow_tb1` computes a trailing pose 0.5 m behind tb1 along its direction of travel and publishes it to `/tb3/goal_pose` at 2 Hz. tb3's Nav2 stack replans continuously to chase this moving goal.

#### Step 5 — Send Additional Waypoints

Repeat Step 4 to send tb1 to new goals. tb3 will continue trailing throughout. There is no need to interact with the tb3 RViz window during normal operation.

***

### Tuning Parameters

All follow behaviour parameters can be overridden as launch arguments:

| Parameter | Default | Effect |
|---|---|---|
| `follow_distance` | `0.5` | Metres tb3 trails behind tb1. Increase if tb3 gets too close |
| `publish_rate` | `2.0` | Hz for goal republishing. Lower reduces replan chatter; higher improves responsiveness |
| `heading_history_size` | `5` | Poses used to estimate tb1's heading. Increase for smoother heading on slow moves |
| `deadband_distance` | `0.1` | Minimum goal displacement (m) before republishing. Reduces unnecessary replanning when tb1 is nearly stationary |

Example — increase follow distance and reduce replan rate:

```bash
docker compose run nav ros2 launch tb3_multi_robot follow_tb3.launch.py \
  follow_distance:=0.8 \
  publish_rate:=1.5
```

***

### Architecture Overview

```
RViz (tb1 window)
  └─► /tb1/goal_pose ──► Nav2 (tb1) ──► tb1 moves

tb1 movement
  └─► /tb1/odom ──► tb3_follow_tb1 node
                        └─► computes trailing pose (0.5 m behind)
                        └─► /tb3/goal_pose ──► Nav2 (tb3, FollowDynamicPoint BT)
                                                   └─► tb3 follows
```

# Original README

## Multi-TurtleBot3 Simulation with ROS 2 Jazzy & Gazebo Harmonic
This repository provides a scalable ROS 2-based framework to simulate multiple TurtleBot3 robots in Gazebo with Navigation2 (Nav2) support. Each robot runs within its own namespace, enabling clean separation and interaction-free operation.

The 'master' branch is updated with Jazzy support.  The 'humble' branch includes an implementation that functions with the humble framework, while the 'foxy' branch provides support specifically for ROS2 Foxy.

## Branch Mapping
'master' -> ROS2 Jazzy

'humble' -> ROS2 Humble

'foxy' -> ROS2 Foxy

The Jazzy version features a streamlined multi-robot setup that improves usability and launch flexibility.

## Prerequisites

- **Operating System**: Ubuntu 24.04
- **ROS Version**: [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- **Gazebo Version**: Gazebo Harmonic

Refer to the official ROS2 Jazzy installation guide: [link](https://docs.ros.org/en/jazzy/Installation.html)

### Install Required Dependencies

```
apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    curl \
    ros-jazzy-rmw-implementation \
    ros-jazzy-rmw-cyclonedds-cpp
```

When launching multiple robots with Nav2, the number of DDS participants can quickly exceed the default limit set by CycloneDDS. To avoid participant ID exhaustion, create a configuration file to increase the allowable range:

Create a file with below contents
e.g `$HOME/cyclonedds.xml`
```
<CycloneDDS>
  <Discovery>
    <ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
  </Discovery>
</CycloneDDS>
```

Set the CYCLONEDDS_URI environment variable to point to the XML configuration file. To make this persistent across terminal sessions, add the export command to ~/.bashrc:

```
export CYCLONEDDS_URI=$HOME/cyclonedds.xml
```

## Setup Workspace and Clone Repository

```
$ mkdir -p robot_ws/src
$ cd robot_ws/src

# Clone the master branch of the multi-robot repo
$ git clone  https://github.com/arshadlab/tb3_multi_robot.git -b master

# Initialize the workspace
$ cd robot_ws
$ source /opt/ros/jazzy/setup.bash
$ rosdep install --from-paths src -r -y
```

 It's recommended to download the default model assets to ensure proper rendering and simulation behavior.

```
$ mkdir -p ~/.gazebo/models
$ git clone https://github.com/osrf/gazebo_models ~/.gazebo/models
```

## 🔧 Build the Workspace
After installing dependencies and setting up the workspace, compile the ROS 2 packages using colcon:

```
$ cd robot_ws/
$ colcon build --symlink-install
$ source ./install/setup.bash
```

Then, update the config/robots.yaml file to define the robot setup.
By default, four robots (tb1, tb2, tb3, tb4) are listed, with only tb1 and tb3 enabled. Modify the names, positions, and enabled flags as needed.

## 🚀 Launch the Simulation (Robots Only)
Use the following command to start the Gazebo simulation with the configured TurtleBot3 robots:

```
$ ros2 launch tb3_multi_robot tb3_world.launch.py
```
<img width="1840" height="1004" alt="image" src="https://github.com/user-attachments/assets/68d08e6a-8ab6-4f3d-98b3-504102b96312" />

After the simulation is launched, the system can either proceed with the Nav2 stack for autonomous navigation or use driving nodes for manual or scripted control.
A Python-based turtlebot3_drive script is included, replicating the original C++ node functionality while addressing compatibility issues with ROS 2 Jazzy and Gazebo Harmonic.

## 🚗 Launch Driving Nodes (Optional)
The original turtlebot3_drive application is not fully compatible with ROS 2 Jazzy and Gazebo Harmonic due to message type differences (e.g., use of TwistStamped for the /cmd_vel topic).
To address this, a Python-based equivalent is provided using compatible message types.

To launch the driving node for each robot, use the command below.
While a drive.launch.py file is included for automated multi-robot support, it is still under development and may require manual execution for each robot.

```
$ ./install/tb3_multi_robot/bin/turtlebot3_drive --ros-args -r __ns:=/tb1
```

Replace /tb1 with the appropriate robot namespace (/tb2, /tb3, etc.) as defined in robot configuration.

## 🧭 Launch Navigation2 Stack

With the robots running in Gazebo (via tb3_world.launch.py), the Navigation2 (Nav2) stack can be launched from a separate terminal.

```
$ ros2 launch  tb3_multi_robot tb3_nav2.launch.py
```
This will launch Nav2 nodes for all enabled robots using their respective namespaces.

<img width="2172" height="1721" alt="image" src="https://github.com/user-attachments/assets/d4c9e2ff-9721-4711-8e89-c25acbb3b207" />

The RViz2 panel title has been updated to include the corresponding robot name, making it easier to match each RViz instance with its respective robot in the Gazebo simulation.

### 🧭 Set Initial Pose
With RViz running, the robot’s initial position and orientation can be set using the 2D Pose Estimate button, aligned to its simulated placement and heading.

<img width="2180" height="1676" alt="image" src="https://github.com/user-attachments/assets/6a0c1bb0-32b1-4cf2-ad74-2da1ffba88e7" />

Alternatively, the initial pose can be set programmatically via the command line when the robot’s position and orientation are known from the simulation.

To retrieve the live pose of robots from Gazebo Harmonic, run:

```
$ gz topic -e -t /world/default/pose/info
```
This command lists the poses of all simulated entities. Identify the target robot by its name, such as tb1_waffle or tb1_burger.

Update the sample commands below with the position and orientation values obtained. The covariance typically remains unchanged.
Below commands are given for included robots.  User will need to update them for any custom robots. 

```
# TB1
$ ros2 topic pub --once /tb1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: -1.500653720729433, y: -0.5000000060919606, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
               
# TB2
$ ros2 topic pub --once /tb2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: -1.500653720729433, y: 0.49999999390803895, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
   
# TB3
$ ros2 topic pub --once /tb3/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: 1.499346279270567, y: -0.50000000609196049, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
  
# TB4
$ ros2 topic pub --once /tb4/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: 1.499346279270567, y: 0.499999993908039, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
```

### 🎯 Set Navigation Goal

A navigation goal may be sent using the 'Nav2 Goal' button in RViz after launching the Nav2 stack.

<img width="1091" height="842" alt="image" src="https://github.com/user-attachments/assets/99e0ebc3-09e8-47e6-be67-5743ca5e6d15" />

Alternatively, goals can be sent programmatically via the command line using ROS 2 action interface.

Below is an example command for setting a goal for tb1:

```
$ ros2 action send_goal /tb1/navigate_to_pose nav2_msgs/action/NavigateToPose \
'{
  pose: {
    header: {
      frame_id: "map"
    },
    pose: {
      position: {
        x: 2.0,
        y: 0.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    }
  }
}'
```

Replace /tb1 with the appropriate robot namespace (e.g., /tb2, /tb3, etc.) and modify the x, y, and orientation fields to specify the desired goal pose.

## RQT Usage
rqt is a versatile tool for inspecting various ROS 2 data. In a multi-robot setup where each robot operates within its own namespace, specific configurations are required for correct usage.
Ensure that the /tf and /tf_static topics are mapped using absolute names (prefixed with /). Additionally, launch rqt within the desired robot's namespace to correctly visualize and interact with its respective topics.

```
rqt --ros-args -r __ns:=/tb1 -r /tf:=tf -r /tf_static:=tf_static
```

<img width="1630" height="1572" alt="image" src="https://github.com/user-attachments/assets/a8f4221b-705e-4b52-ab04-03916a60de08" />


## 🐳 Running via Dockers
A Dockerfile is provided to simplify the setup and execution of multi-robot simulation using ROS 2 Jazzy. This enables running the project even on systems other than Ubuntu 24.04 (e.g. Ubuntu 22.04) without requiring ROS installation on the host. The Docker image includes all necessary dependencies preconfigured.

### 🛠️ Build Docker image

Clone the repository and build the Docker image:

```
$ git clone  https://github.com/arshadlab/tb3_multi_robot.git -b jazzy
$ cd docker
$ docker build -t tb3_multi_robot:jazzy .
```

This will build container and also clone and build the repo in /opt/ros2_ws.

### 🚀 Launch robots in Gazebo

Run the container and launch the Gazebo simulation:

```
$ docker run -it --rm \
  --user $(id -u):$(id -g) \
  --name tb3sim \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/dev/dri:/dev/dri" \
  tb3_multi_robot:jazzy \
  ros2 launch tb3_multi_robot tb3_world.launch.py
```

Ensure the command `xhost +local:docker` is executed on the host system to permit GUI display access for Docker containers.


### Launch Nav2 nodes in already running container

After the robots are active, open a new terminal and run:

```
$ docker exec -it tb3sim bash -c "
  source /opt/ros2_ws/install/setup.bash && \
  ros2 launch tb3_multi_robot tb3_nav2.launch.py
"
```

This launches the Nav2 stack inside the already running container.

## Accessing host project folder via Docker.

To build and run the multi-robot demo using a local (host-cloned) repository inside the Docker environment, mount the host directory into the container. Note that the default /opt/ros2_ws workspace inside the container will not be used in this case.

```
$ docker run -it --rm \
  --user $(id -u):$(id -g) \
  --name tb3sim \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/dev/dri:/dev/dri" \
  --volume="<absolute path of local project/workspace>:/robot_ws" \
  tb3_multi_robot:jazzy \
  bash
```
Note: Replace < absolute path to local project/workspace > with the full path to the cloned tb3_multi_robot directory on the host system.

This will open an interactive shell. Navigate to /robot_ws inside the container and build the workspace using colcon. All changes will reflect directly in the host directory.

To access the running container from another terminal, use:

```
$ docker exec -it tb3sim bash
```

Note: By default, this will place the shell in /opt/ros2_ws. Change directory to /robot_ws manually and source ./install/setup.bash from there.
When building both inside and outside the container, avoid using --symlink-install to prevent conflicts.

## Robot Configuration

The placement and activation of individual robots are defined in the config/robots.yaml file. Each robot is assigned a unique name and initial position. Set the enabled flag to true to include a robot in the simulation or false to exclude it.

Below is an example configuration used in the Nav2 simulation:

```
 robots:
  - name: tb1
    x_pose: "-1.5"
    y_pose: "-0.5"
    z_pose: 0.01
    enabled: true
  - name: tb2
    x_pose: "-1.5"
    y_pose: "0.5"
    z_pose: 0.01
    enabled: false
  - name: tb3
    x_pose: "1.5"
    y_pose: "-0.5"
    z_pose: 0.01
    enabled: true
  - name: tb4
    x_pose: "1.5"
    y_pose: "0.5"
    z_pose: 0.01
    enabled: false
```
💡 Robots can be enabled or disabled by updating the enabled field, and their starting poses can be configured by modifying the position values accordingly.

## Improving performance

Simulating multiple robots (especially 4 or more) can be demanding on system resources. Below are some suggestions to help optimize performance:

### 1. Limit Robot Count
Running fewer robots significantly reduces CPU and memory load. Consider limiting the simulation to 2 robots if performance is a concern.

### 2. Lower Simulation Update Rates
Reduce the update frequency in the Gazebo .world file to ease the physics computation load. In worlds/tb3_world.world, modify the <physics> block as follows:

```
<physics type="ode">
      <real_time_update_rate>100.0</real_time_update_rate>
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
```

🔧 Lower real_time_update_rate and higher max_step_size lead to fewer simulation steps per second, improving runtime performance.

### 3. Reduce Topic Frequency
Consider modifying the robot model or relevant plugins to reduce the frequency of published topics (e.g., /odom, /tf, /scan) if they are not critical at high rates.

# FAQ
**Why are /tf and /tf_static explicitly remapped in RViz and other nodes, even when a namespace is applied?**

Although nodes like rviz2 can be launched with a specific namespace, the TransformListener in tf2_ros subscribes to the global topics /tf and /tf_static by default. This behavior stems from how the listener is implemented internally — the topic names are hardcoded with a leading slash, making them absolute paths (refer to /opt/ros/jazzy/include/tf2_ros/transform_listener.h for reference).

This can be quickly verified using the following command:

```
rviz2 --ros-args -r __ns:=/tb1
```

Despite the namespace, the resulting RViz instance still subscribes to /tf and /tf_static globally.

In a multi-robot configuration, where each robot is designed to operate with an isolated TF tree, remapping /tf to tf (a relative topic) ensures proper namespacing. This approach prevents conflicts and guarantees that TF messages remain within the intended robot scope.

Such remapping is also necessary in other components (e.g., Nav2) that instantiate their own transform listeners. Applying consistent remapping avoids unintended cross-robot data mixing and supports clean separation of transform data across all robot instances.

# 📎 Note on Included Files

Some of configuration and model files (e.g., from turtlebot3 and nav2) have been directly copied into this repository. These were modified to better suit the multi-robot simulation and to ensure long-term consistency and reproducibility—even if the original upstream repositories evolve or change in the future. All original credit for these files remains with their respective authors and maintainers.

</details>
