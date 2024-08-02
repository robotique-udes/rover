#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rover_msgs.msg import ArmCmd
import math as m
from math import pi as PI
import numpy as np
import matplotlib.pyplot as plt

class SimulationStatus(Node):
    
    def __init__(self):
        super().__init__("simulation_status")
        self.simulation_status = self.create_subscription(
            ArmCmd, "/rover/arm/cmd/in/teleop", self.simulationCallback, 10)
        
        self.fig = plt.figure(figsize=(8, 8))
        self.ax_top = self.fig.add_subplot(2, 2, 1)
        self.ax_3d = self.fig.add_subplot(2, 2, 2, projection='3d')
        self.ax_front = self.fig.add_subplot(2, 2, 3)
        self.ax_right = self.fig.add_subplot(2, 2, 4)
        plt.ion()
        plt.show()
        
    def simulationCallback(self, ArmMsg: ArmCmd):
        
        self.JL_pos = ArmMsg.position[ArmCmd.JL]
        self.J0_pos = ArmMsg.position[ArmCmd.J0]
        self.J1_pos = ArmMsg.position[ArmCmd.J1]
        self.J2_pos = ArmMsg.position[ArmCmd.J2]
        self.GripperTilt_pos = ArmMsg.position[ArmCmd.GRIPPERTILT]
        
        qPosition = np.array([self.JL_pos, self.J0_pos, self.J1_pos, self.J2_pos, self.GripperTilt_pos])
        pointPos = self.computeDirectKin(qPosition)
        self.plot(pointPos)
        
    def computeDirectKin(self, qPosition):
        pointPos = np.zeros((6, 3))
        
        J0x = 0.0
        J0y = 0.0
        J0z = 0.0
        
        J1x = 0.0
        J1y = 0.0
        J1z = 0.0
        
        J2x = 0.650
        J2y = 0.0
        J2z = 0.0
        
        J3x = 0.625
        J3y = 0.0
        J3z = 0.0
        
        J4x = 0.217
        J4y = 0.0
        J4z = 0.0
        
        q0 = qPosition[0]
        q1 = qPosition[1]
        q2 = qPosition[2]
        q3 = qPosition[3]
        q4 = qPosition[4]
        
        pointPos[0, 0] = 0.0
        pointPos[0, 1] = q0
        pointPos[0, 2] = 0.0
        
        pointPos[1, 0] = J0x
        pointPos[1, 1] = J0y + q0
        pointPos[1, 2] = J0z
        
        pointPos[2, 0] = J0x + J1x * m.cos(q1) - J1y * m.sin(q1)
        pointPos[2, 1] = J0y + q0 + J1x * m.sin(q1) + J1y * m.cos(q1)
        pointPos[2, 2] = J0z + J1z
        
        pointPos[3, 0] = J0x + J1x * m.cos(q1) + J2x * m.cos(q1) * m.cos(0.5 * PI - q2) + J2z * m.cos(q1) * m.sin(0.5 * PI - q2) - m.sin(q1) * (J1y + J2y)
        pointPos[3, 1] = J0y + q0 + J1x * m.sin(q1) + m.cos(q1) * (J1y + J2y) + J2x * m.sin(q1) * m.cos(0.5 * PI - q2) + J2z * m.sin(q1) * m.sin(0.5 * PI - q2)
        pointPos[3, 2] = J0z + J1z + J2z * m.cos(0.5 * PI - q2) + J2x * m.sin(0.5 * PI - q2)
        
        pointPos[4, 0] = J0x + J1x * m.cos(q1) + J2x * m.cos(q1) * m.cos(0.5 * PI - q2) + J2z * m.cos(q1) * m.sin(0.5 * PI - q2) + J3x * m.cos(q1) * m.cos(0.5 * PI - q2 - q3) + J3z * m.cos(q1) * m.sin(0.5 * PI - q2 - q3) - m.sin(q1) * (J1y + J2y + J3y)
        pointPos[4, 1] = J0y + q0 + J1x * m.sin(q1) + m.cos(q1) * (J1y + J2y) + J2x * m.sin(q1) * m.cos(0.5 * PI - q2) + J2z * m.sin(q1) * m.sin(0.5 * PI - q2) + J3x * m.sin(q1) * m.cos(0.5 * PI - q2 - q3) + J3z * m.sin(q1) * m.sin(0.5 * PI - q2 - q3)
        pointPos[4, 2] = J0z + J1z + J2z * m.cos(0.5 * PI - q2) + J3z * m.cos(0.5 * PI - q2 - q3) + J2x * m.sin(0.5 * PI - q2) + J3x * m.sin(0.5 * PI - q2 - q3)  
        
        pointPos[5, 0] = J0x + J1x * m.cos(q1) + J2x * m.cos(q1) * m.cos(0.5 * PI - q2) + J2z * m.cos(q1) * m.sin(0.5 * PI - q2) + J3x * m.cos(q1) * m.cos(0.5 * PI - q2 - q3) + J3z * m.cos(q1) * m.sin(0.5 * PI - q2 - q3) - m.sin(q1) * (J1y + J2y) + J4x * m.cos(q1) * m.cos(0.5 * PI - q2 - q3 - q4) + J4z * m.cos(q1) * m.sin(0.5 * PI - q2 - q3 - q4)
        pointPos[5, 1] = J0y + q0 + J1x * m.sin(q1) + m.cos(q1) * (J1y + J2y) + J2x * m.sin(q1) * m.cos(0.5 * PI - q2) + J2z * m.sin(q1) * m.sin(0.5 * PI - q2) + J3x * m.sin(q1) * m.cos(0.5 * PI - q2 - q3) + J3z * m.sin(q1) * m.sin(0.5 * PI - q2 - q3) + J4x * m.sin(q1) * m.cos(0.5 * PI - q2 - q3 - q4) + J4z * m.sin(q1) * m.sin(0.5 * PI - q2 - q3 - q4)
        pointPos[5, 2] = J0z + J1z + J2z * m.cos(0.5 * PI - q2) + J3z * m.cos(0.5 * PI - q2 - q3) + J4z * m.cos(0.5 * PI - q2 - q3 - q4) + J2x * m.sin(0.5 * PI - q2) + J3x * m.sin(0.5 * PI - q2 - q3) + J4x * m.sin(0.5 * PI - q2 - q3 - q4)  

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
    node = SimulationStatus()
    rclpy.spin(node)
    rclpy.shutdown()