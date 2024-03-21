import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from custom_messages.msg import KinData
from custom_messages.msg import CartesianState
from .position_interpolation.position_interpolation import PositionInterpolation
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.declare_parameter('control_gain', 1e2)
        self.publisher_     = self.create_publisher(JointState, 'desired_joint_velocity', 10)
        self.subs_kin_data  = self.create_subscription(
            KinData,
            'kin_data',
            self.listener_callback_kin_data,
            10)
        self.subs_desired_state = self.create_subscription(
            CartesianState,
            'desired_state',
            self.listener_callback_desired_state,
            10)
        self.subs_kin_data
        self.subs_desired_state
        self.initialized        = False
        self.running            = False
        self.pi                 = np.zeros(2)
        self.pf                 = np.zeros(2)
        self.sampling_period    = 1e-2

    def listener_callback_kin_data(self, kin_data):
        if self.initialized == False:
            self.timer          = self.create_timer(self.sampling_period, self.timer_callback)
            self.pd_dot         = np.array([0.0,0.0])
            self.pd             = np.array([kin_data.x,kin_data.y])
            self.initialized    = True
        self.p                  = np.array([kin_data.x,kin_data.y])
        self.jacobian           = np.array([[kin_data.jacobian[0], kin_data.jacobian[1]],
                                            [kin_data.jacobian[2],kin_data.jacobian[3]]])

    def listener_callback_desired_state(self, desired_state):
        if self.running == False:
            self.running    = True
        self.pd             = np.array([desired_state.x,desired_state.y])
        self.pd_dot         = np.array([desired_state.xdot,desired_state.ydot])

    def timer_callback(self):
        if self.running:
            control_gain    = self.get_parameter('control_gain').value
            qdot_d          = np.linalg.inv(self.jacobian) @ ( (self.pd - self.p) * control_gain + self.pd_dot )
        else:
            qdot_d          = np.zeros(2)
        joint_state                 = JointState()
        now                         = self.get_clock().now()
        joint_state.header.stamp    = now.to_msg()
        joint_state.name            = ['r1', 'r2']
        joint_state.velocity        = [qdot_d[0], qdot_d[1]]
        self.publisher_.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
