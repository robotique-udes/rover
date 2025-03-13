#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math as m
from math import pi as PI
import numpy as np
import matplotlib.pyplot as plt

from rover_msgs.msg import ArmMsg 

class ArmSimulation(Node):
    
    def __init__(self):
        super().__init__("arm_simulation")
        
        self.goal_velocity = self.create_subscription(
            ArmMsg, "/rover/arm/cmd/goal_speed", self.goalVelocityCallback, 10)        
        
        self.current_position_publisher = self.create_publisher(
            ArmMsg, "/rover/arm/status/current_positions", 10)
                
        self.fig = plt.figure(figsize=(8, 8))
        self.ax_top = self.fig.add_subplot(2, 2, 1)
        self.ax_3d = self.fig.add_subplot(2, 2, 2, projection='3d')
        self.ax_front = self.fig.add_subplot(2, 2, 3)
        self.ax_right = self.fig.add_subplot(2, 2, 4)
        plt.ion()
        plt.show()

        

        self.JL_pos = 0.0
        self.J1_pos = 0.0
        self.J2_pos = PI / 2
        self.Gripper = 0.0

        self.linearJointVelocity = 0.0
        self.shoulderJointVelocity = 0.0
        self.elbowJointVelocity = 0.0
        self.gripperJointVelocity = 0.0

        self.dt = 0.1

    def goalVelocityCallback(self, msg):

        self.linearJointVelocity = msg.data[msg.JL]
        self.shoulderJointVelocity = msg.data[msg.J1]
        self.elbowJointVelocity = msg.data[msg.J2]
        self.gripperJointVelocity = msg.data[msg.GRIPPER_TILT]

        self.JL_pos += self.linearJointVelocity * self.dt
        self.J1_pos += self.shoulderJointVelocity * self.dt
        self.J2_pos += self.elbowJointVelocity * self.dt
        self.Gripper += self.gripperJointVelocity * self.dt

        qPosition = np.array([self.JL_pos, self.J1_pos, self.J2_pos, self.Gripper])
        pointPos = self.computeDirectKin(qPosition)

        self.plot(pointPos)

        self.publish_joint_positions()

    def publish_joint_positions(self):
        msg = ArmMsg()
        msg.data = [
            self.JL_pos,
            self.J1_pos,
            self.J2_pos,
            self.Gripper,
            0.0,
            0.0
        ]

        self.current_position_publisher.publish(msg)

        qPosition = np.array(msg.data)
        pointPos = self.computeDirectKin(qPosition)
        self.plot(pointPos)

    def computeDirectKin(self, qPosition):
        pointPos = np.zeros((4, 3))
        
        J1 = 0.435
        J2 = 0.371
        J3 = 0.185
        
        q0 = qPosition[0]
        q1 = qPosition[1]
        q2 = qPosition[2]
        q3 = qPosition[3]
        
        pointPos[0, 0] = q0
        pointPos[0, 1] = 0.0
        pointPos[0, 2] = 0.0        
        
        pointPos[1, 0] = q0
        pointPos[1, 1] = J1 * m.sin(q1)
        pointPos[1, 2] = J1 * m.cos(q1)
        
        pointPos[2, 0] = q0
        pointPos[2, 1] = J1 * m.sin(q1) + J2 * m.sin(q1 + q2)
        pointPos[2, 2] = J1 * m.cos(q1) + J2 * m.cos(q1 + q2)
        
        pointPos[3, 0] = q0
        pointPos[3, 1] = J1 * m.sin(q1) + J2 * m.sin(q1 + q2) + J3 * m.sin(q1 + q2+ q3)
        pointPos[3, 2] = J1 * m.cos(q1) + J2 * m.cos(q1 + q2) + J3 * m.cos(q1 + q2+ q3)

        return pointPos

    def plot(self, pointPos):
        xs = pointPos[:, 0]
        ys = pointPos[:, 1]
        zs = pointPos[:, 2]
        
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
    node = ArmSimulation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()