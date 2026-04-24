"""
maze_sim.launch.py — ROS 2 Humble · Gazebo Classic 11 · TurtleBot3 Burger
==========================================================================

KEY FIX: The TurtleBot3 Burger has TWO model files:

  1. turtlebot3_description/urdf/turtlebot3_burger.urdf
     → Visual geometry + joints only. NO Gazebo plugins.
     → Spawning this gives you a robot that LOOKS right but has
       no /scan, no /odom, no /cmd_vel.

  2. turtlebot3_gazebo/models/turtlebot3_burger/model.sdf  ← CORRECT
     → Full SDF with diff_drive plugin, LiDAR plugin, IMU plugin.
     → Spawning this gives you all three topics automatically.

This launch file uses method 2 (the SDF), matching how TurtleBot3's
own spawn_turtlebot3.launch.py does it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package paths ────────────────────────────────────────────────
    pkg_maze      = get_package_share_directory('maze_navigation')
    pkg_gz_ros    = get_package_share_directory('gazebo_ros')
    pkg_tb3_gz    = get_package_share_directory('turtlebot3_gazebo')
    pkg_tb3_desc  = get_package_share_directory('turtlebot3_description')

    # ── File paths ───────────────────────────────────────────────────
    world_file = os.path.join(pkg_maze, 'worlds', 'simple_maze.world')

    # The SDF model that contains ALL Gazebo plugins (diff_drive, LiDAR, IMU)
    sdf_model = os.path.join(
        pkg_tb3_gz, 'models', 'turtlebot3_burger', 'model.sdf'
    )

    # The URDF is still needed for robot_state_publisher (TF tree)
    urdf_file = os.path.join(
        pkg_tb3_desc, 'urdf', 'turtlebot3_burger.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # ── Launch arguments ─────────────────────────────────────────────
    x_arg = DeclareLaunchArgument('x_pose', default_value='0.5')
    y_arg = DeclareLaunchArgument('y_pose', default_value='0.5')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # ── 0. TURTLEBOT3_MODEL env var ──────────────────────────────────
    set_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', value='burger'
    )

    # ── 1. Gazebo Classic (server + client) ──────────────────────────
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # ── 2. Robot State Publisher (for TF tree) ───────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc,
        }],
    )

    # ── 3. Spawn robot using the SDF model (has all plugins) ─────────
    #        This is exactly how turtlebot3_gazebo's own
    #        spawn_turtlebot3.launch.py does it: -file pointing to
    #        the model.sdf, NOT the URDF.
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', sdf_model,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
        ],
        output='screen',
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        set_model,
        start_gazebo,
        robot_state_pub,
        spawn_robot,
    ])
