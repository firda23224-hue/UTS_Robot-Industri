#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class PointtoPoint_Nav(Node):
    def __init__(self):
        super().__init__(
            'point_to_point_navigasi',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/robot_position', self.pose_callback, 10)

        # parameter (sesuaikan dengan kemampuan robot/simulasi Anda)
        self.max_linear = 2.0
        self.min_linear = 0.04
        self.max_angular = 1.5
        self.min_angular = 0.15
        self.k_linear = 1.5
        self.k_angular = 3.0
        self.goal_tolerance = 0.04
        self.rotate_threshold = 0.25
        self.align_tolerance = 0.05
        self.max_linear_accel = 2.0
        self.max_angular_accel = 1.5
        self.straight_min_length = 1.0

        self.dt = 0.05
        self.pos_x = self.pos_y = self.pos_theta = 0.0
        self.pose_received = False

        self.waypoints = [
            (0.50, 2.30), 
            (2.40, 2.50), 
            (2.40, 3.10), 
            (3.65, 3.10),
            (3.65, 0.85), 
            (7.95, 0.85), 
            (7.95, 6.12), 
            (7.33, 7.08),
            (6.10, 7.30), 
            (5.20, 6.80),
            (5.00, 6.50), 
            (4.85, 4.30),
            (4.00, 4.30), 
            (3.90, 6.80), 
            (0.95, 7.55), 
            (0.25, 7.55),
            (0.25, 4.80),
        ]
        self.waypoint_index = 0
        self.segment_start = None
        self.v = 0.0          # kecepatan yang sedang diperintahkan
        self.omega = 0.0
        self.rotating = False

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("navigasi point to point ready")
        self.get_logger().info(f"waypoints: {self.waypoints}")


    def pose_callback(self, msg):
        self.pos_x = msg.linear.x
        self.pos_y = msg.linear.y
        self.pos_theta = msg.angular.z
        if not self.pose_received:
            self.pose_received = True
            self.segment_start = (self.pos_x, self.pos_y)

    def publish_velocity(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.cmd_vel_publisher.publish(msg)

    def command_velocity(self, v, omega):
        # batasi percepatan agar roda tidak slip
        self.v += clamp(v - self.v, self.max_linear_accel * self.dt)
        self.omega += clamp(omega - self.omega, self.max_angular_accel * self.dt)
        self.publish_velocity(self.v, self.omega)

    def control_loop(self):
        if not self.pose_received:
            return

        # semua waypoint selesai: berhenti halus, lalu matikan timer
        if self.waypoint_index >= len(self.waypoints):
            self.command_velocity(0.0, 0.0)
            if self.v == 0.0 and self.omega == 0.0:
                self.get_logger().info("semua waypoint selesai --> finish")
                self.timer.cancel()
            return

        x, y, theta = self.pos_x, self.pos_y, self.pos_theta
        goal_x, goal_y = self.waypoints[self.waypoint_index]
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy)

        seg_x = goal_x - self.segment_start[0]
        seg_y = goal_y - self.segment_start[1]
        segment_length = math.hypot(seg_x, seg_y)

        # tetap melaju di belokan landai, berhenti kalau harus putar di tempat
        corner_speed = 0.0
        if self.waypoint_index + 1 < len(self.waypoints) and segment_length > 0.0:
            nx, ny = self.waypoints[self.waypoint_index + 1]
            turn = normalize_angle(math.atan2(ny - goal_y, nx - goal_x)
                                   - math.atan2(seg_y, seg_x))
            if abs(turn) < self.rotate_threshold:
                corner_speed = self.max_linear

        # waypoint dianggap tercapai begitu terlewati
        passed = corner_speed > 0.0 and dx * seg_x + dy * seg_y <= 0.0

        if distance < self.goal_tolerance or passed:
            self.get_logger().info(
                f"waypoint {self.waypoint_index} tercapai, "
                f"pos=({x:.2f}, {y:.2f}) target=({goal_x}, {goal_y})")
            self.segment_start = (goal_x, goal_y)
            self.waypoint_index += 1
            return

        heading_error = normalize_angle(math.atan2(dy, dx) - theta)
        omega = clamp(self.k_angular * heading_error, self.max_angular)

        if abs(heading_error) > self.rotate_threshold:
            self.rotating = True
        elif abs(heading_error) < self.align_tolerance:
            self.rotating = False

        if self.rotating:
            if abs(omega) < self.min_angular:
                omega = math.copysign(self.min_angular, omega)
            self.command_velocity(0.0, omega)
            return

        # batas pengereman agar sempat melambat ke corner_speed
        brake_speed = math.sqrt(corner_speed ** 2
                                + 2.0 * self.max_linear_accel * distance)
        approach_speed = max(self.k_linear * distance, corner_speed)
        v = min(self.max_linear, approach_speed, brake_speed)
        v = max(v, self.min_linear)
        self.command_velocity(v, omega)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = PointtoPoint_Nav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()