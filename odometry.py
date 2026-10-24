#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],)

        # Sub untuk membaca kecepatan roda dari topik /wheel/speed
        self.subscription = self.create_subscription(Float64MultiArray, '/wheel/speed', self.wheel_speed_callback, 10)

        # Pub hasil global koordinat
        self.position_publisher = self.create_publisher(Twist, '/robot_position', 10)

        # Parameter robot
        self.radius_roda = 0.03  # radius roda
        self.sumbu_roda = 0.17  # jarak antar roda

        # Posisi awal robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Waktu sebelumnya untuk menghitung delta t (dt)
        self.last_time = None

        self.get_logger().info("Odometry Node Ready")

    def wheel_speed_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn("Pesan /wheel/speed harus memiliki dua nilai")
            return

        # Membaca kecepatan sudut roda dari topik (rad/s)
        omega_left = msg.data[0]
        omega_right = msg.data[1]

        # Konversi kecepatan sudut roda ke kecepatan linear (v = omega*r)
        v_left = omega_left * self.radius_roda
        v_right = omega_right * self.radius_roda

        # Forward kinematics kecepatan linear (Vb) dan kecepatan angular (omega) robot
        vb = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.sumbu_roda

        # Hitung selisih waktu (dt)
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            return

        # Global koordinat
        delta_theta = omega * dt
        heading_midpoint = self.theta + (delta_theta / 2.0)
        
        # Update posisi global menggunakan sudut tengah
        self.x += vb * dt * math.cos(heading_midpoint)
        self.y += vb * dt * math.sin(heading_midpoint)
        self.theta += delta_theta

        pos_msg = Twist()
        pos_msg.linear.x = self.x
        pos_msg.linear.y = self.y
        pos_msg.angular.z = self.theta

        # Publish
        self.position_publisher.publish(pos_msg)

        
        #tampilin logger posisi robot di terminal
        self.get_logger().info(
        f"posisi -> x: {self.x:.2f} m, y: {self.y:.2f} m, "
        f"theta: {math.degrees(self.theta):.2f}",
        throttle_duration_sec=0.5,)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


        
        