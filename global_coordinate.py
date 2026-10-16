#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


class GlobalCoordinate(Node):

    def __init__(self):
        super().__init__('global_coordinate')

        # Ukuran robot dari file XACRO
        self.wheel_radius = 0.1
        self.wheel_separation = 0.45

        # Menyimpan data kecepatan roda dan arah robot
        self.left_wheel_velocity = None
        self.right_wheel_velocity = None
        self.theta = None

        # Membaca kecepatan roda dari Gazebo
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Membaca orientasi robot dari TF
        self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.calculate_global_velocity
        )

        self.get_logger().info(
            'Global Coordinate aktif: membaca data robot dari Gazebo'
        )

    def joint_state_callback(self, msg):

        try:
            left_index = msg.name.index('base_left_wheel_joint')
            right_index = msg.name.index('base_right_wheel_joint')
        except ValueError:
            return

        if (
            left_index < len(msg.velocity)
            and right_index < len(msg.velocity)
        ):
            self.left_wheel_velocity = msg.velocity[left_index]
            self.right_wheel_velocity = msg.velocity[right_index]

    def tf_callback(self, msg):

        for transform in msg.transforms:

            if (
                transform.header.frame_id == 'odom'
                and transform.child_frame_id == 'base_footprint'
            ):
                q = transform.transform.rotation

                # Mengubah quaternion menjadi sudut yaw (theta)
                self.theta = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                break

    def calculate_global_velocity(self):

        # Tunggu sampai data robot diterima
        if (
            self.left_wheel_velocity is None
            or self.right_wheel_velocity is None
            or self.theta is None
        ):
            self.get_logger().info(
                'Menunggu data roda dan orientasi robot dari Gazebo...'
            )
            return

        phi_l = self.left_wheel_velocity
        phi_r = self.right_wheel_velocity

        # Menghitung kecepatan linear robot
        V_B = self.wheel_radius * (phi_r + phi_l) / 2.0

        # Menghitung kecepatan sudut robot
        omega = (
            self.wheel_radius
            * (phi_r - phi_l)
            / self.wheel_separation
        )

        # Rumus Global Coordinate
        x_dot = V_B * math.cos(self.theta)
        y_dot = V_B * math.sin(self.theta)
        theta_dot = omega

        # Menampilkan hasil di terminal
        self.get_logger().info(
            '\n========== GLOBAL COORDINATE ==========\n'
            f'Kecepatan roda kiri  : {phi_l:.3f} rad/s\n'
            f'Kecepatan roda kanan : {phi_r:.3f} rad/s\n'
            f'V_B                  : {V_B:.3f} m/s\n'
            f'Theta                : {self.theta:.3f} rad\n'
            f'Omega                : {omega:.3f} rad/s\n'
            f'X_dot                : {x_dot:.3f} m/s\n'
            f'Y_dot                : {y_dot:.3f} m/s\n'
            f'Theta_dot            : {theta_dot:.3f} rad/s\n'
            '======================================='
        )


def main(args=None):

    rclpy.init(args=args)
    node = GlobalCoordinate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()