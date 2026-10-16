#!/usr/bin/env python3
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class InverseKinematics(Node):

    def __init__(self, linear_velocity, angular_velocity):
        super().__init__('inverse_kinematics')

        # Parameter robot
        self.wheel_radius = 0.03       # meter
        self.wheel_separation = 0.17   # meter

        # Input kecepatan robot
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

        # Publisher ke Gazebo melalui /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publisher kecepatan roda [phi_L, phi_R] untuk node global coordinate
        self.wheel_publisher = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity',
            10
        )

        # Hitung inverse kinematics (sekali, karena input konstan)
        self.phi_left, self.phi_right = self.inverse_kinematics()

        # Publish berulang 10 Hz agar node global coordinate
        # menerima data secara kontinu
        self.timer = self.create_timer(0.1, self.publish_command)

    def inverse_kinematics(self):

        r = self.wheel_radius
        s = self.wheel_separation

        # Matriks Forward Kinematics
        kinematics_matrix = np.array([
            [r / 2,  r / 2],
            [-r / s, r / s]
        ])

        # Inverse matriks
        inverse_matrix = np.linalg.inv(kinematics_matrix)

        velocity_vector = np.array([
            self.linear_velocity,
            self.angular_velocity
        ])

        # Inverse Kinematics
        wheel_velocity = inverse_matrix @ velocity_vector

        phi_left = float(wheel_velocity[0])
        phi_right = float(wheel_velocity[1])

        self.get_logger().info(
            f'Input: V = {self.linear_velocity:.3f} m/s, '
            f'omega = {self.angular_velocity:.3f} rad/s'
        )
        self.get_logger().info(f'phi_L = {phi_left:.3f} rad/s')
        self.get_logger().info(f'phi_R = {phi_right:.3f} rad/s')

        return phi_left, phi_right

    def publish_command(self):

        # Kirim perintah ke Gazebo melalui /cmd_vel
        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = self.angular_velocity
        self.cmd_vel_publisher.publish(cmd)

        # Kirim kecepatan roda ke node global coordinate
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [self.phi_left, self.phi_right]
        self.wheel_publisher.publish(wheel_msg)


def main(args=None):

    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print(
            '\nCara menjalankan:\n'
            'ros2 run robin_bringup inverse_kinematics '
            '<linear_velocity> <angular_velocity>\n\n'
            'Contoh:\n'
            'ros2 run robin_bringup inverse_kinematics 0.1 0.0\n'
        )
        rclpy.shutdown()
        return

    try:
        linear_velocity = float(sys.argv[1])
        angular_velocity = float(sys.argv[2])
    except ValueError:
        print('Input harus berupa angka.')
        rclpy.shutdown()
        return

    node = InverseKinematics(linear_velocity, angular_velocity)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Hentikan robot saat node dimatikan (Ctrl+C)
        stop = Twist()
        node.cmd_vel_publisher.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()