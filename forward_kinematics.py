#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist


class ForwardKinematics(Node):

    def __init__(self):
        super().__init__('forward_kinematics')

        # Parameter robot
        self.wheel_radius = 0.03       # meter
        self.wheel_separation = 0.17   # meter

        # Kecepatan roda
        self.phi_left = 0.0
        self.phi_right = 0.0

        # Subscriber joint states dari Gazebo
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher hasil Forward Kinematics
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/forward_kinematics',
            10
        )

        self.get_logger().info(
            'Forward Kinematics node started'
        )

    def joint_state_callback(self, msg):

        # Cari posisi index masing-masing roda
        try:
            left_index = msg.name.index('base_left_wheel_joint')
            right_index = msg.name.index('base_right_wheel_joint')
        except ValueError:
            self.get_logger().warn(
                'Wheel joint belum ditemukan pada /joint_states'
            )
            return

        # Ambil kecepatan roda
        if len(msg.velocity) > left_index:
            self.phi_left = msg.velocity[left_index]

        if len(msg.velocity) > right_index:
            self.phi_right = msg.velocity[right_index]

        # Parameter
        r = self.wheel_radius
        s = self.wheel_separation

        
        # Forward Kinematics

        kinematics_matrix = np.array([
            [r / 2, r / 2],
            [-r / s, r / s]
        ])

        wheel_velocity = np.array([
            self.phi_left,
            self.phi_right
        ])

        # Hitung kecepatan robot
        robot_velocity = kinematics_matrix @ wheel_velocity

        linear_velocity = robot_velocity[0]
        angular_velocity = robot_velocity[1]

        # Buat message Twist
        msg_velocity = Twist()

        msg_velocity.linear.x = float(linear_velocity)
        msg_velocity.angular.z = float(angular_velocity)

        # Publish
        self.velocity_publisher.publish(msg_velocity)

        # Tampilkan hasil
        self.get_logger().info(
            f'phi_L = {self.phi_left:.3f} rad/s, '
            f'phi_R = {self.phi_right:.3f} rad/s | '
            f'V = {linear_velocity:.3f} m/s, '
            f'omega = {angular_velocity:.3f} rad/s'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ForwardKinematics()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
