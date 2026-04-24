================================================================================
  ROS 2 Final Project — Maze Navigation using Potential Field Method
  README: Build and Launch Instructions
================================================================================

Prerequisites:
  - Ubuntu 22.04 (VirtualBox)
  - ROS 2 Humble installed
  - Robot: TurtleBot3 Burger
  - Simulator: Gazebo Classic 11

================================================================================
Step 1: Install Dependencies
================================================================================

  sudo apt update
  sudo apt install -y ros-humble-gazebo-ros-pkgs \
                      ros-humble-turtlebot3 \
                      ros-humble-turtlebot3-simulations \
                      ros-humble-robot-state-publisher

================================================================================
Step 2: Source ROS 2 and Set Environment
================================================================================

  source /opt/ros/humble/setup.bash
  export TURTLEBOT3_MODEL=burger

================================================================================
Step 3: Build the Workspace
================================================================================

  cd ~/ros2_project_ws
  colcon build --symlink-install
  source install/setup.bash

================================================================================
Step 4: Launch Simulation (Terminal 1)
================================================================================

  export TURTLEBOT3_MODEL=burger
  source /opt/ros/humble/setup.bash
  source ~/ros2_project_ws/install/setup.bash
  ros2 launch maze_navigation maze_sim.launch.py

  Wait until you see: "SpawnEntity: Successfully spawned entity"

================================================================================
Step 5: Verify Topics (Terminal 2)
================================================================================

  source /opt/ros/humble/setup.bash
  source ~/ros2_project_ws/install/setup.bash
  ros2 topic list | grep -E "cmd_vel|odom|scan"

  Expected output:
    /cmd_vel
    /odom
    /scan

================================================================================
Step 6: Run the Planner (Terminal 3)
================================================================================

  source /opt/ros/humble/setup.bash
  source ~/ros2_project_ws/install/setup.bash
  ros2 run maze_navigation potential_field_planner --ros-args \
    -p goal_x:=9.0 \
    -p goal_y:=9.0 \
    -p k_att:=1.5 \
    -p max_linear_vel:=0.3 \
    -p max_angular_vel:=2.0

  The robot will navigate from (0.5, 0.5) to (9.0, 9.0).
  It stops when "GOAL REACHED" appears in the terminal.

================================================================================
Step 7: Stopping the Simulation
================================================================================

  Press Ctrl+C in Terminal 1 to stop Gazebo.
  Press Ctrl+C in Terminal 3 to stop the planner.

================================================================================
Notes
================================================================================

  - The project skeleton was written for Jazzy + Gz Sim, but our system
    runs Humble + Gazebo Classic. Key adaptations:
      * Removed Ignition plugins from .world file, downgraded SDF to 1.6
      * No ros_gz_bridge needed (Gazebo Classic plugins publish natively)
      * Uses Twist (not TwistStamped) on /cmd_vel
      * Spawns the SDF model (not URDF) to get sensor plugins

  - If Gazebo shows "not responding", click Wait — it can take 2-3 minutes
    on VirtualBox due to 3D rendering.

  - To run without the Gazebo GUI (faster on VirtualBox):
      ros2 launch maze_navigation maze_sim.launch.py gui:=false

================================================================================
