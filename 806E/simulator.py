import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from custom_messages.msg import KinData
from custom_messages.msg import CartesianState
from .position_interpolation.position_interpolation import PositionInterpolation
import numpy as np

class Simulator(Node):
    def __init__(self):
        super().__init__('simulator')
        self.publisher_                     = self.create_publisher(
            JointState, 
            'joint_states', 
            10)
        self.subs_desired_joint_velocity    = self.create_subscription(
            JointState,
            'desired_joint_velocity',
            self.listener_callback_desired,
            10)
        self.subs_desired_joint_velocity
        self.initialized        = False
        self.running            = False
        self.pi                 = np.zeros(2)
        self.pf                 = np.zeros(2)
        self.sampling_period    = 1e-2
        self.q                  = np.array([0, np.pi/2])
        self.qdot_d             = np.zeros(2)
        self.timer              = self.create_timer(self.sampling_period, self.timer_callback)
        
    def listener_callback_desired(self, desired_joint_velocity):
        self.qdot_d             = np.array([desired_joint_velocity.velocity[0],desired_joint_velocity.velocity[1]])

    def timer_callback(self):
        self.q                      = self.q + (self.qdot_d) * self.sampling_period
        joint_state                 = JointState()
        now                         = self.get_clock().now()
        joint_state.header.stamp    = now.to_msg()
        joint_state.name            = ['r1', 'r2']
        joint_state.position        = [self.q[0], self.q[1]]
        self.publisher_.publish(joint_state)
        
def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator()
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
