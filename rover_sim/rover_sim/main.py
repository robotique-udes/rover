import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import numpy as np

class KinematicSubscriber(Node):
    def __init__(self):
        super().__init__('kinematic_subscriber')
        self.subscription_jl = self.create_subscription(Float64, '_currentJLPos', self.listener_callback, 10)
        self.subscription_j0 = self.create_subscription(Float64, '_currentJ0Pos', self.listener_callback, 10)
        self.subscription_j1 = self.create_subscription(Float64, '_currentJ1Pos', self.listener_callback, 10)
        self.subscription_j2 = self.create_subscription(Float64, '_currentJ2Pos', self.listener_callback, 10)
        self.subscription_gripper_tilt = self.create_subscription(Float64, '_currentGripperTilt', self.listener_callback, 10)
        
        self.jl_pos = None
        self.j0_pos = None
        self.j1_pos = None
        self.j2_pos = None
        self.gripper_tilt = None

    def listener_callback(self, msg):
        topic = msg._topic_name
        if topic == '_currentJLPos':
            self.jl_pos = msg.data
        elif topic == '_currentJ0Pos':
            self.j0_pos = msg.data
        elif topic == '_currentJ1Pos':
            self.j1_pos = msg.data
        elif topic == '_currentJ2Pos':
            self.j2_pos = msg.data
        elif topic == '_currentGripperTilt':
            self.gripper_tilt = msg.data
        
        if None not in (self.jl_pos, self.j0_pos, self.j1_pos, self.j2_pos, self.gripper_tilt):
            self.compute_direct_kinematic()

    def compute_direct_kinematic(self):
        # Perform your calculations here
        # Example calculation (to be replaced with actual computations):
        x = [self.jl_pos, self.j0_pos, self.j1_pos, self.j2_pos]
        y = [self.gripper_tilt, self.j1_pos, self.j2_pos, self.jl_pos]

        self.plot_points(x, y)
        
    def plot_points(self, x, y):
        plt.figure()
        plt.plot(x, y, marker='o')
        plt.title('Kinematic Points')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.show()
        
def main(args=None):
    rclpy.init(args=args)
    kinematic_subscriber = KinematicSubscriber()
    rclpy.spin(kinematic_subscriber)
    kinematic_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
