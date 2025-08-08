#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from rover_msgs.msg import ArmMsg

class ArmVisualization(Node):
    def __init__(self):
        super().__init__("arm_visualization")
        self.sub = self.create_subscription(
            ArmMsg, "/rover/arm/joints_status", self.cb, 10)
        self.fig = plt.figure(figsize=(8, 8))
        self.ax_top = self.fig.add_subplot(2, 2, 1)
        self.ax_3d = self.fig.add_subplot(2, 2, 2, projection='3d')
        self.ax_front = self.fig.add_subplot(2, 2, 3)
        self.ax_right = self.fig.add_subplot(2, 2, 4)
        plt.ion()
        plt.show()

    def cb(self, msg):
        # Use the correct indices according to your message definition!
        q = np.array([
            msg.current_position[0],  # JL
            msg.current_position[2],  # J1 (Shoulder)
            msg.current_position[3],  # J2 (Elbow)
            msg.current_position[4],  # Gripper tilt
        ])
        points = self.fk(q)
        self.plot(points)

    def fk(self, q):
        # Forward kinematics with 90° offset at J2
        J1 = 0.435
        J2 = 0.371
        J3 = 0.185
        q0, q1, q2, q3 = q
        points = np.zeros((4, 3))
        # Base
        points[0] = [q0, 0.0, 0.0]
        # Shoulder
        points[1] = [q0,
                     J1 * np.sin(q1),
                     J1 * np.cos(q1)]
        # Elbow (90° offset at J2)
        points[2] = [q0,
                     J1 * np.sin(q1) + J2 * np.sin(q1 + q2 + np.pi/2),
                     J1 * np.cos(q1) + J2 * np.cos(q1 + q2 + np.pi/2)]
        # Gripper
        points[3] = [q0,
                     J1 * np.sin(q1) + J2 * np.sin(q1 + q2 + np.pi/2) + J3 * np.sin(q1 + q2 + q3 + np.pi/2),
                     J1 * np.cos(q1) + J2 * np.cos(q1 + q2 + np.pi/2) + J3 * np.cos(q1 + q2 + q3 + np.pi/2)]
        return points

    def plot(self, p):
        xs, ys, zs = p[:, 0], p[:, 1], p[:, 2]
        self.ax_top.clear()
        self.ax_3d.clear()
        self.ax_front.clear()
        self.ax_right.clear()

        self.ax_top.plot(xs, ys, marker='o')
        self.ax_top.set_title("Top view (x-y plane)")
        self.ax_top.set_xlabel('x')
        self.ax_top.set_ylabel('y')
        self.ax_top.set_xlim(-2, 2)
        self.ax_top.set_ylim(-2, 2)
        self.ax_top.grid(True)

        self.ax_3d.plot(xs, ys, zs, marker='o')
        self.ax_3d.set_title("3D view")
        self.ax_3d.set_xlabel('x')
        self.ax_3d.set_ylabel('y')
        self.ax_3d.set_zlabel('z')
        self.ax_3d.set_xlim3d(-2, 2)
        self.ax_3d.set_ylim3d(-2, 2)
        self.ax_3d.set_zlim3d(-2, 2)
        self.ax_3d.grid(True)

        self.ax_front.plot(ys, zs, marker='o')
        self.ax_front.set_title("Front view (y-z plane)")
        self.ax_front.set_xlabel('y')
        self.ax_front.set_ylabel('z')
        self.ax_front.set_xlim(-2, 2)
        self.ax_front.set_ylim(-2, 2)
        self.ax_front.grid(True)

        self.ax_right.plot(xs, zs, marker='o')
        self.ax_right.set_title("Right view (x-z plane)")
        self.ax_right.set_xlabel('x')
        self.ax_right.set_ylabel('z')
        self.ax_right.set_xlim(-2, 2)
        self.ax_right.set_ylim(-2, 2)
        self.ax_right.grid(True)

        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = ArmVisualization()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()