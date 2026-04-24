# Maze Navigation using Potential Field Method

> A ROS 2 autonomous robot that navigates through a maze using Artificial Potential Field (APF) with a smart local-minima escape strategy.

**ROS 2 Humble** · **Gazebo Classic 11** · **TurtleBot3 Burger** · **Python**

---

## What This Does

A TurtleBot3 Burger navigates from point A to point B inside a walled maze — avoiding obstacles in real-time using LiDAR and the Potential Field Method. When the robot gets trapped in a local minimum (forces cancel out at wall corners), it automatically detects the trap and escapes using intermediate waypoints.

---

## How It Works

### Artificial Potential Field (APF)

The robot is treated as a particle in a force field:

| Force | Formula | Effect |
|-------|---------|--------|
| **Attractive** | `F = k_att × (goal - robot)` | Pulls toward the goal |
| **Repulsive** | `F = k_rep × (1/d - 1/d_obs) / d²` | Pushes away from walls |

The sum of all forces determines heading and speed.

### Wall Avoidance

- Walls within `0.6m` generate repulsive force — closer = stronger push
- Speed drops to **50%** within `0.4m`, **20%** within `0.2m`
- Robot rotates in place when the target is behind it

### Local Minima Escape

**Detection:** The maze is divided into `0.5m × 0.5m` grid cells. If the robot visits the same cell **200+ times** while within **5m of the goal**, it's stuck.

**Escape:** The planner switches to intermediate waypoints that route around the blocking walls, then resumes normal APF navigation.

---

## 🗂️ Project Structure

```
maze_navigation/
├── package.xml                  # ROS 2 package manifest
├── setup.py                     # Python package setup
├── setup.cfg
├── launch/
│   └── maze_sim.launch.py       # Launches Gazebo + robot
├── worlds/
│   ├── simple_maze.world        # 10×10m maze (3 inner walls)
│   └── complex_maze.world       # 12×12m bonus maze
├── maze_navigation/
│   ├── __init__.py
│   └── potential_field_planner.py   # APF algorithm
└── resource/
    └── maze_navigation
```

---

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11

### Install Dependencies

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs \
                    ros-humble-turtlebot3 \
                    ros-humble-turtlebot3-simulations \
                    ros-humble-robot-state-publisher
```

### Build

```bash
cd ~/ros2_project_ws
colcon build --symlink-install
source install/setup.bash
```

### Run

```bash
# Terminal 1 — Launch simulation
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source ~/ros2_project_ws/install/setup.bash
ros2 launch maze_navigation maze_sim.launch.py

# Terminal 2 — Run the planner
source /opt/ros/humble/setup.bash
source ~/ros2_project_ws/install/setup.bash
ros2 run maze_navigation potential_field_planner --ros-args \
  -p goal_x:=9.0 -p goal_y:=9.0
```

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal_x` | 9.0 | Goal X coordinate |
| `goal_y` | 9.0 | Goal Y coordinate |
| `k_att` | 2.0 | Attractive force gain |
| `k_rep` | 80.0 | Repulsive force gain |
| `d_obs` | 0.6 m | Obstacle influence radius |
| `max_linear_vel` | 0.5 m/s | Maximum forward speed |
| `max_angular_vel` | 3.0 rad/s | Maximum turn speed |
| `goal_tolerance` | 0.4 m | Stop distance from goal |

---

## Humble vs Jazzy — Key Differences

This project was adapted from a Jazzy skeleton to run on Humble:

| | Jazzy | Humble |
|---|---|---|
| **Simulator** | Gz Sim (Harmonic) | Gazebo Classic 11 |
| **Bridge** | `ros_gz_bridge` required | Not needed — plugins publish natively |
| **Velocity topic** | `TwistStamped` | `Twist` |
| **SDF version** | 1.8 | 1.6 |
| **Robot model** | URDF | SDF (includes Gazebo plugins) |

---

## Topics Used

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Publish (velocity commands) |
| `/odom` | `nav_msgs/msg/Odometry` | Subscribe (robot position) |
| `/scan` | `sensor_msgs/msg/LaserScan` | Subscribe (LiDAR data) |
