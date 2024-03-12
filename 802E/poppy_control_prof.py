#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import math
import numpy as np
import matplotlib.pyplot as plt  # MATLAB plotting functions


# Constants
pi = math.pi
tf = 40.0


l_arm = ['l_shoulder_y', 'l_shoulder_x', 'l_arm_z', 'l_elbow_y']

class PoppyController(Node):
    def __init__(self):
        super().__init__('poppy_controller')

        self.get_logger().info("Starting the controller")

        self.declare_parameter("type", "robot type, either humanoid (full robot) or arms (left and right arms only)")

        self.robot_type_ = str(self.get_parameter("type").value)

        self.cmd_publisher_ = self.create_publisher(Float64MultiArray, '/joint_group_position_controller/commands', 10)

        self.joint_state_subscription_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        joint_count = 25 if self.robot_type_ == "humanoid" else 8
        self.cmd_ = Float64MultiArray()
        for _ in range(joint_count):
            self.cmd_.data.append(0)

        self.wait_for_initial_position = True

    # Called after the a JointState message arrives if self.wait_for_initial_position is True
    def init(self):

        # set a non-singular initial pose
        self.set_motor_position('l_shoulder_y', 0.0*pi/180.0)           # Attention : l_shoulder_y est supposé être le premier moteur or pour la commande set_motor_position, la commande attaque le moteur 3
        self.set_motor_position('l_shoulder_x', 0.0*pi/180.0)           # l-shoulder_x attaque le moteur 4
        self.set_motor_position('l_arm_z', 90.0*pi/180.0)               # l_arm_z attaque le moteur 2
        self.set_motor_position('l_elbow_y', 90.0*pi/180.0)             # l_elbow_y attaque le moteur  1 
        # attention la commande get_moteur_position est bien parameté, càd que get_motor_position('l_shoulder_y') récupère la valeur du premier moteur
        
        

    
        self.cmd_publisher_.publish(self.cmd_)

        # Wait until the initial pose is reached
        while True:
            error = 0.0
            #for i in range(len(self.cmd_.data)):
            #    error += math.fabs(self.cmd_.data[i] -self.joint_positions_[i])
           
            # le calcul de l'erreur a été modifié pour se conformer aux remarques précédentes --> si la commande set_motor_position est réglé, on peut revenir à la définition de l'erreur ci-dessus
            error = math.fabs(self.cmd_.data[0] - self.joint_positions_[2]) + math.fabs(self.cmd_.data[1] - self.joint_positions_[3]) + math.fabs(self.cmd_.data[2] - self.joint_positions_[1]) + math.fabs(self.cmd_.data[3] - self.joint_positions_[0])    
            
            if error < 0.01:
                break
            else:
                return

        self.wait_for_initial_position = False

        self.t0 = self.get_time()
        
        self.get_logger().info("Reached initial joint position (%s), starting the control loop" %
                               self.joint_positions_)

        
        self.index = 0
        
        # Init done, now start the control loop
        self.run_timer = self.create_timer(0.1, self.run)

    def run(self):
        
        dt = self.get_time() - self.t0
        
        if dt < tf:
            
            # TODO : Ecrire ici votre boucle de commande
        
            self.get_logger().info("t = %s" %dt)

            

            self.cmd_publisher_.publish(self.cmd_)
        else:
            self.plot()

    def plot(self):

        self.destroy_node()

    def calc_mgd(self):
        """
        :return X: vecteur contenant la position de l'organe terminal
        """

        



    def get_motor_position(self, joint_name):
        index = self.joint_index(joint_name)



        return self.joint_positions_[index]

    def set_motor_position(self, joint_name, joint_pos):
        index = self.joint_index(joint_name)


        
        self.cmd_.data[index] = joint_pos

    def joint_state_callback(self, msg):
        self.joint_names_ = msg.name
        self.joint_positions_ = msg.position
        if self.wait_for_initial_position:
            self.init()

    def joint_index(self, joint_name):
        return self.joint_names_.index(joint_name)

    def get_time(self):
        sec_nsec = self.get_clock().now().seconds_nanoseconds()
        return sec_nsec[0] + 1e-9 * sec_nsec[1]


def main(args=None):
    rclpy.init(args=args)

    poppy_controller = PoppyController()

    try:
        rclpy.spin(poppy_controller)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#  poppy control pour commande ré
