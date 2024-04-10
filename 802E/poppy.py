#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import numpy as np
import matplotlib.pyplot as plt  # Not focusing on plotting functions

# Constants
pi = math.pi
tf = 40.0
a1, a2, a3, a4, a5, a6, a7, a8, a9 = 0.0771, 0.05, 0.004, 0.0284, 0.03625, 0.0185, 0.11175, -0.01, 0.15
Kp = np.diag([1.0, 1.0, 1.0])  # Proportional gain
t = np.zeros((1,1000000))  # Time array
Xi = np.zeros((3, 1))  # Initial position
Xf = np.array([[0.2615], [0.1740], [0.0540]])  # Final desired position
x_t = np.zeros((3,1000000))
x_d = np.zeros((3,1000000))
qd = np.zeros((4,2))
q_t = np.zeros((4,1000000))
q_td = np.zeros((4,1000000))


l_arm = ['l_shoulder_y', 'l_shoulder_x', 'l_arm_z', 'l_elbow_y']  # Joint names

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

        global Xi
        Xi = self.calc_mgd()
        self.index = 0
        qd[0, 0] = self.get_motor_position(l_arm[0])
        qd[1, 0] = self.get_motor_position(l_arm[1])
        qd[2, 0] = self.get_motor_position(l_arm[2])
        qd[3, 0] = self.get_motor_position(l_arm[3])
    
        t[0, self.index] = 0
        t[0, self.index+1] = t[0, self.index]
        # Init done, now start the control loop
        self.run_timer = self.create_timer(0.1, self.run)



    def plot(self):
        # Use dictionaries to store data and labels for cyclic processing
        data_sets = {
            'Position réelle en x': x_t[0, 0:self.index],
            'Position désirée en x': x_d[0, 0:self.index],
            'Position réelle en y': x_t[1, 0:self.index],
            'Position désirée en y': x_d[1, 0:self.index],
            'Position réelle en z': x_t[2, 0:self.index],
            'Position désirée en z': x_d[2, 0:self.index],
        }

        motor_positions = {
            'Position moteur l_elbow_y': q_t[0, 0:self.index],
            'Position moteur l_arm_z': q_t[1, 0:self.index],
            'Position moteur l_shoulder_y': q_t[2, 0:self.index],
            'Position moteur l_shoulder_x': q_t[3, 0:self.index],
        }

        motor_speeds = {
            'Vitesse moteur l_elbow_y': q_td[0, 0:self.index],
            'Vitesse moteur l_arm_z': q_td[1, 0:self.index],
            'Vitesse moteur l_shoulder_y': q_td[2, 0:self.index],
            'Vitesse moteur l_shoulder_x': q_td[3, 0:self.index],
        }

        # Charting locations
        plt.figure(1)
        plt.suptitle('Position de l\'organe terminal en fonction du temps')
        for label, data in data_sets.items():
            plt.plot(t[0, 0:self.index], data, label=label)
        plt.legend()
        plt.xlabel('Temps (s)')
        plt.ylabel('Position (m)')

        # Charting Angles
        plt.figure(2)
        plt.suptitle('Position angulaire des moteurs en fonction du temps')
        for label, data in motor_positions.items():
            plt.plot(t[0, 0:self.index], data, label=label)
        plt.legend()
        plt.xlabel('Temps (s)')
        plt.ylabel('Angle (rad)')

        # Plotting speed charts
        plt.figure(3)
        plt.suptitle('Variation de la vitesse angulaire en fonction du temps')
        for label, data in motor_speeds.items():
            plt.plot(t[0, 0:self.index], data, label=label)
        plt.legend()
        plt.xlabel('Temps (s)')
        plt.ylabel('Vitesse (rad/s)')

        plt.show()

        self.destroy_node()




    def calc_mgd(self):
        # Retrieve the current positions of the joints
        q1 = self.get_motor_position('l_shoulder_y')
        q2 = self.get_motor_position('l_shoulder_x')
        q3 = self.get_motor_position('l_arm_z')
        q4 = self.get_motor_position('l_elbow_y')

        # Pre-compute cosines and sines of the joint angles to simplify equations
        c1, c2, c3, c4 = np.cos(q1), np.cos(q2), np.cos(q3), np.cos(q4)
        s1, s2, s3, s4 = np.sin(q1), np.sin(q2), np.sin(q3), np.sin(q4)

        # Apply forward kinematics formulas to calculate the end-effector position
        x = (c2 * c4 + s2 * s3 * s4) * a9 + c2 * a7 + s2 * s3 * a8 + c2 * a5 + a4 + a1
        y = (-s1 * s2 * c4 + (s1 * c2 * s3 - c1 * c3) * s4) * a9 - s1 * s2 * a7 + (s1 * c2 * s3 - c1 * c3) * a8 - s1 * s2 * a5 - c1 * a6 + a2
        z = (c1 * s2 * c4 + (-c1 * c2 * s3 - s1 * c3) * s4) * a9 + c1 * s2 * a7 + (-c1 * c2 * s3 - s1 * c3) * a8 + c1 * s2 * a5 - s1 * a6 + a3

        # Return the calculated position of the end-effector
        return x, y, z


    def Jacob(self, Kp):
        """
        Calculate the Jacobian inverse matrix for proportional control.

        Parameters:
        - Kp: Proportional gain matrix.

        Returns:
        - Ja: Adjusted Jacobian matrix used for calculating joint velocities.
        """
    
        # Retrieve the current joint angles
        q1 = self.get_motor_position(l_arm[0])
        q2 = self.get_motor_position(l_arm[1])
        q3 = self.get_motor_position(l_arm[2])
        q4 = self.get_motor_position(l_arm[3])
        
        # Define constants for matrix elements to improve readability
        c2, c3, c4 = np.cos(q2), np.cos(q3), np.cos(q4)
        s2, s3, s4 = np.sin(q2), np.sin(q3), np.sin(q4)
        c1, s1 = np.cos(q1), np.sin(q1)
        
        # Calculate the Jacobian matrix based on the robot's kinematics
        J = np.array([
            [0, -0.15 * s2 * c4 - 0.148 * s2 + 0.15 * s3 * s4 * c2 - 0.01 * s3 * c2, 0.15 * s2 * s4 * c3 - 0.01 * s2 * c3, 0.15 * s2 * s3 * c4 - 0.15 * s4 * c2],
            [(0.15 * s1 * c3 + 0.15 * s3 * c1 * c2) * s4 - 0.01 * s1 * c3 + 0.0185 * s1 - 0.15 * s2 * c1 * c4 - 0.148 * s2 * c1 - 0.01 * s3 * c1 * c2, -0.15 * s1 * s2 * s3 * s4 + 0.01 * s1 * s2 * s3 - 0.15 * s1 * c2 * c4 - 0.148 * s1 * c2, (0.15 * s1 * c2 * c3 + 0.15 * s3 * c1) * s4 - 0.01 * s1 * c2 * c3 - 0.01 * s3 * c1, (0.15 * s1 * s3 * c2 - 0.15 * c1 * c3) * c4 + 0.15 * s1 * s2 * s4],
            [(0.15 * s1 * s3 * c2 - 0.15 * c1 * c3) * s4 - 0.15 * s1 * s2 * c4 - 0.148 * s1 * s2 - 0.01 * s1 * s3 * c2 + 0.01 * c1 * c3 - 0.0185 * c1, 0.15 * s2 * s3 * s4 * c1 - 0.01 * s2 * s3 * c1 + 0.15 * c1 * c2 * c4 + 0.148 * c1 * c2, (0.15 * s1 * s3 - 0.15 * c1 * c2 * c3) * s4 - 0.01 * s1 * s3 + 0.01 * c1 * c2 * c3, (-0.15 * s1 * c3 - 0.15 * s3 * c1 * c2) * c4 - 0.15 * s2 * s4 * c1]
        ])
        
        # Compute the pseudo-inverse of the Jacobian
        Jt = np.linalg.pinv(J)
        
        # Adjust the Jacobian with the proportional gain
        Ja = Jt @ Kp
        
        return Ja


    def Gen_traj(self, Xi, xd, yd, zd, dt, tf):
        """
         Generate a trajectory from initial position Xi to the target position (xd, yd, zd) based on the current time dt and the final time tf.

        :param Xi: Initial position as a numpy array or list [xi, yi, zi]
        :param xd: Target x position
        :param yd: Target y position
        :param zd: Target z position
        :param dt: Current time
        :param tf: Total time of the trajectory
        :return: The new position as a numpy array [xt, yt, zt]
         """

        if not 0 <= dt <= tf:
            raise ValueError("Current time dt should be within the range [0, tf]")

        Dx = xd - Xi[0]
        Dy = yd - Xi[1]
        Dz = zd - Xi[2]

        rt = 10 * (dt/tf)**3 - 15 * (dt/tf)**4 + 6 * (dt/tf)**5

        xt = Xi[0] + rt *Dx
        yt = Xi[1] + rt *Dy
        zt = Xi[2] + rt *Dz

        return xt,yt,zt

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

    def run(self):

        dt = self.get_time() - self.t0
        t[0, self.index+1] = dt

        if dt < tf:
            xt,yt,zt = self.Gen_traj(Xi,0.2615,0.1749,0.0540,dt,tf)
            Xd = np.array([[xt],[yt],[zt]])

            x, y, z = self.calc_mgd()
            X = np.array([[x],[y],[z]])

            Xp = Kp @ (Xd - X)
            qp = self.Jacob(Xp)

            for i in range(4):
              
                qd[i, 1] = qp[i]*(t[0, self.index+1] - t[0, self.index]) + qd[i, 0] 
                qd[i, 0] = qd[i, 1]

            self.set_motor_position('l_elbow_y', qd[0, 1])  
            self.set_motor_position('l_arm_z', qd[1, 1]) 
            self.set_motor_position('l_shoulder_y', qd[2, 1]) 
            self.set_motor_position('l_shoulder_x', qd[3, 1])



            t[0, self.index] = dt
    
            x_t[0, self.index] = X[0]
            x_t[1, self.index] = X[1]
            x_t[2, self.index] = X[2]

            x_d[0, self.index] = Xd[0]
            x_d[1, self.index] = Xd[1]
            x_d[2, self.index] = Xd[2]
    
            q_t[0, self.index] = qd[0, 0]
            q_t[1, self.index] = qd[1, 0]
            q_t[2, self.index] = qd[2, 0]
            q_t[3, self.index] = qd[3, 0]
        
            q_td[0, self.index] = qp[0, 0]
            q_td[1, self.index] = qp[1, 0]
            q_td[2, self.index] = qp[2, 0]
            q_td[3, self.index] = qp[3, 0]
    
    
            self.index = self.index+1
    
            self.cmd_publisher_.publish(self.cmd_)
        else:
            self.plot()
    
    
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
