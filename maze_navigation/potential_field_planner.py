#!/usr/bin/env python3
"""
Potential Field planner for maze navigation.
ROS 2 Humble - Gazebo Classic - TurtleBot3 Burger
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class PotentialFieldPlanner(Node):

    def __init__(self):
        super().__init__('potential_field_planner')

        # Parameters
        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 9.0)
        self.declare_parameter('k_att', 2.0)
        self.declare_parameter('k_rep', 80.0)
        self.declare_parameter('d_obs', 0.6)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 3.0)
        self.declare_parameter('goal_tolerance', 0.4)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.k_att = self.get_parameter('k_att').value
        self.k_rep = self.get_parameter('k_rep').value
        self.d_obs = self.get_parameter('d_obs').value
        self.max_v = self.get_parameter('max_linear_vel').value
        self.max_w = self.get_parameter('max_angular_vel').value
        self.goal_tol = self.get_parameter('goal_tolerance').value

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.ranges = None
        self.angles = None
        self.ready = False

        # Stuck detection: visit count grid
        self.visits = {}
        self.cell_size = 0.5
        self.visit_limit = 200

        # Waypoint escape
        self.mode = 'apf'  # 'apf' or 'waypoint'
        self.waypoints = []
        self.wp_idx = 0

        # Publishers / Subscribers
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y})')

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.ready = True

    def scan_cb(self, msg):
        r = np.array(msg.ranges, dtype=np.float64)
        bad = np.isnan(r) | np.isinf(r) | (r <= 0)
        r[bad] = msg.range_max if np.isfinite(msg.range_max) else 30.0
        self.ranges = r
        if self.angles is None:
            self.angles = np.linspace(msg.angle_min, msg.angle_max, len(r))

    def apf(self, tx, ty):
        """Compute APF velocity toward target (tx, ty)."""
        dx = tx - self.x
        dy = ty - self.y

        # Attractive
        fx = self.k_att * dx
        fy = self.k_att * dy

        # Repulsive
        for i, d in enumerate(self.ranges):
            if d >= self.d_obs:
                continue
            a = self.yaw + self.angles[i]
            ox = self.x + d * math.cos(a)
            oy = self.y + d * math.sin(a)
            ax, ay = self.x - ox, self.y - oy
            dist = math.hypot(ax, ay)
            if dist < 1e-6:
                continue
            mag = self.k_rep * (1/d - 1/self.d_obs) / (d*d)
            fx += mag * ax / dist
            fy += mag * ay / dist

        # Force -> velocity
        heading = math.atan2(fy, fx)
        err = math.atan2(math.sin(heading - self.yaw), math.cos(heading - self.yaw))

        cmd = Twist()
        cmd.angular.z = max(-self.max_w, min(self.max_w, 2.5 * err))

        align = math.cos(err)
        cmd.linear.x = self.max_v * align if align > 0 else 0.0

        # Slow down near walls
        closest = float(np.min(self.ranges))
        if closest < 0.2:
            cmd.linear.x *= 0.2
        elif closest < 0.4:
            cmd.linear.x *= 0.5

        return cmd

    def loop(self):
        if not self.ready or self.ranges is None:
            return

        goal_d = math.hypot(self.goal_x - self.x, self.goal_y - self.y)

        # Goal reached
        if goal_d < self.goal_tol:
            self.pub.publish(Twist())
            self.timer.cancel()
            self.get_logger().info(f'GOAL REACHED ({self.x:.1f}, {self.y:.1f})')
            return

        # Waypoint mode
        if self.mode == 'waypoint':
            wx, wy = self.waypoints[self.wp_idx]
            wd = math.hypot(wx - self.x, wy - self.y)
            if wd < 0.5:
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    self.mode = 'apf'
                    self.visits.clear()
                    self.get_logger().info('Waypoints done, back to APF')
                    return
            cmd = self.apf(wx, wy)
            self.pub.publish(cmd)
            self.get_logger().info(
                f'WP[{self.wp_idx+1}/{len(self.waypoints)}] '
                f'({self.x:.1f},{self.y:.1f}) d={wd:.1f}',
                throttle_duration_sec=1.0)
            return

        # Stuck detection
        cell = (int(self.x / self.cell_size), int(self.y / self.cell_size))
        self.visits[cell] = self.visits.get(cell, 0) + 1
        if self.visits[cell] >= self.visit_limit and goal_d < 5.0:
            self.mode = 'waypoint'
            self.waypoints = [(5.0, 1.0), (8.5, 1.0)]
            self.wp_idx = 0
            self.get_logger().warn(f'Stuck! Following waypoints')
            return

        # Normal APF
        cmd = self.apf(self.goal_x, self.goal_y)
        self.pub.publish(cmd)
        self.get_logger().info(
            f'({self.x:.1f},{self.y:.1f}) d={goal_d:.1f} '
            f'v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
