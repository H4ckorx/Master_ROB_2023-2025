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
a1 = 0.0771
a2 = 0.05
a3 = 0.004
a4 = 0.0284
a5 = 0.03625
a6 = 0.0185
a7 = 0.11175
a8 = -0.01
a9 = 0.15
t = np.zeros((1,1000000))
Kp = np.diag([1.0, 1.0, 1.0])
qd = np.zeros((4,2))
Xi = np.zeros((3, 1))
Xf = np.array([[0.2615], [0.1740], [0.0540]])
x_t = np.zeros((3,1000000))
x_d = np.zeros((3,1000000))
q_t = np.zeros((4,1000000))
q_td = np.zeros((4,1000000))




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
    def update_motor_positions(self, qd):
    # Setting the position of all joints
        for i, name in enumerate(l_arm):
            self.set_motor_position(name, qd[i])

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
        q1 = self.get_motor_position(l_arm[0])
        q2 = self.get_motor_position(l_arm[1])
        q3 = self.get_motor_position(l_arm[2])
        q4 = self.get_motor_position(l_arm[3])

        x = (np.cos(q2)*np.cos(q4)+np.sin(q2)*np.sin(q3)*np.sin(q4))*a9+np.cos(q2)*a7+np.sin(q2)*np.sin(q3)*a8+np.cos(q2)*a5+a4+a1
        y = (-np.sin(q1)*np.sin(q2)*np.cos(q4)+(np.sin(q1)*np.cos(q2)*np.sin(q3)-np.cos(q1)*np.cos(q3))*np.sin(q4))*a9-np.sin(q1)*np.sin(q2)*a7+(np.sin(q1)*np.cos(q2)*np.sin(q3)-np.cos(q1)*np.cos(q3))*a8-np.sin(q1)*np.sin(q2)*a5-np.cos(q1)*a6+a2
        z = (np.cos(q1)*np.sin(q2)*np.cos(q4)+(-np.cos(q1)*np.cos(q2)*np.sin(q3)-np.sin(q1)*np.cos(q3))*np.sin(q4))*a9+np.cos(q1)*np.sin(q2)*a7+(-np.cos(q1)*np.cos(q2)*np.sin(q3)-np.sin(q1)*np.cos(q3))*a8+np.cos(q1)*np.sin(q2)*a5-np.sin(q1)*a6+a3

        

        return x,y,z


    def Jacob(self,q1,q2,q3,q4):
        J = np.array([
        [0, 
         -0.15*np.sin(q2)*np.cos(q4) - 0.148*np.sin(q2) + 0.15*np.sin(q3)*np.sin(q4)*np.cos(q2) - 0.01*np.sin(q3)*np.cos(q2), 
         0.15*np.sin(q2)*np.sin(q4)*np.cos(q3) - 0.01*np.sin(q2)*np.cos(q3), 
         0.15*np.sin(q2)*np.sin(q3)*np.cos(q4) - 0.15*np.sin(q4)*np.cos(q2)],
        
        [(0.15*np.sin(q1)*np.cos(q3) + 0.15*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.sin(q4) - 0.01*np.sin(q1)*np.cos(q3) + 0.0185*np.sin(q1) - 0.15*np.sin(q2)*np.cos(q1)*np.cos(q4) - 0.148*np.sin(q2)*np.cos(q1) - 0.01*np.sin(q3)*np.cos(q1)*np.cos(q2), 
         -0.15*np.sin(q1)*np.sin(q2)*np.sin(q3)*np.sin(q4) + 0.01*np.sin(q1)*np.sin(q2)*np.sin(q3) - 0.15*np.sin(q1)*np.cos(q2)*np.cos(q4) - 0.148*np.sin(q1)*np.cos(q2), 
         (0.15*np.sin(q1)*np.cos(q2)*np.cos(q3) + 0.15*np.sin(q3)*np.cos(q1))*np.sin(q4) - 0.01*np.sin(q1)*np.cos(q2)*np.cos(q3) - 0.01*np.sin(q3)*np.cos(q1), 
         (0.15*np.sin(q1)*np.sin(q3)*np.cos(q2) - 0.15*np.cos(q1)*np.cos(q3))*np.cos(q4) + 0.15*np.sin(q1)*np.sin(q2)*np.sin(q4)],
        
        [(0.15*np.sin(q1)*np.sin(q3)*np.cos(q2) - 0.15*np.cos(q1)*np.cos(q3))*np.sin(q4) - 0.15*np.sin(q1)*np.sin(q2)*np.cos(q4) - 0.148*np.sin(q1)*np.sin(q2) - 0.01*np.sin(q1)*np.sin(q3)*np.cos(q2) + 0.01*np.cos(q1)*np.cos(q3) - 0.0185*np.cos(q1), 
         0.15*np.sin(q2)*np.sin(q3)*np.sin(q4)*np.cos(q1) - 0.01*np.sin(q2)*np.sin(q3)*np.cos(q1) + 0.15*np.cos(q1)*np.cos(q2)*np.cos(q4) + 0.148*np.cos(q1)*np.cos(q2), 
         (0.15*np.sin(q1)*np.sin(q3) - 0.15*np.cos(q1)*np.cos(q2)*np.cos(q3))*np.sin(q4) - 0.01*np.sin(q1)*np.sin(q3) + 0.01*np.cos(q1)*np.cos(q2)*np.cos(q3), 
         (-0.15*np.sin(q1)*np.cos(q3) - 0.15*np.sin(q3)*np.cos(q1)*np.cos(q2))*np.cos(q4) - 0.15*np.sin(q2)*np.sin(q4)*np.cos(q1)]
        ])
        return J

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

        D = np.array([xd, yd, zd]) - Xi
        rt = 10 * (dt/tf)**3 - 15 * (dt/tf)**4 + 6 * (dt/tf)**5

        Xt = Xi + rt * D

        return Xt

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

