#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from casadi import *


class RobotController:
    def __init__(self):
        # Topics
        pub_topic_name = "/jetbot0/diff_drive_controller/cmd_vel"
        sub_topic_name = "/jetbot0/diff_drive_controller/odom"

        # ROS Subscribers and Publishers
        self.velocity_pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10)
        self.velocity_sub = rospy.Subscriber(sub_topic_name, Odometry, self.odometry_callback)
        
        # Command message
        self.velocity_msg = Twist()
        
        
        #controller
        self.controller = None
        self.u_prev = vertcat(0,0)


    def odometry_callback(self ,msg):

        # Extract position and orientation from Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, theta = tf.transformations.euler_from_quaternion(quaternion)
        # Log the extracted values
        rospy.loginfo("Position: x = %.2f, y = %.2f, Theta = %.2f", x, y, theta)

        # State vector as a column vector (3x1)
        state_vector = np.array([[x], [y], [theta]])
        
        # Call the controllerSimple function
        control_output = self.controllerSimple(state_vector)
        self.u_prev = control_output[:,0] 
        
        
        # Assign linear and angular speeds to the Twist message
        self.velocity_msg.linear.x = control_output[0, 0]  # Linear speed
        self.velocity_msg.angular.z = control_output[1, 0]  # Angular speed



        # Log the control actions without non-ASCII characters
        rospy.loginfo("Linear Velocity: {:.2f} m/s, Angular Velocity: {:.2f} rad/s".format(float(control_output[0, 0]), float(control_output[1, 0])))

        # Publish the command
        self.velocity_pub.publish(self.velocity_msg)

    def controllerSimple(self, robot_states):
        if self.controller is None:
             # Initialize constants
            b = 0.22  # Distance between wheels
            v_max = 1  # Maximum speed (m/s)
            a_max = 3  # Maximum acceleration (m/s^2)
            omega_max = 1  # Maximum angular velocity (rad/s)
            alpha_max = 3  # Maximum angular acceleration (rad/s^2)
            T_s = 0.5  # Sampling time
            horizon = 5  # Prediction horizon
            ref = vertcat(10, 10, 0)  # Target reference position
            Q = diag(DM([10, 10, 0]))  # State weights
            R = diag(DM([8, 1]))  # Control input weights
            
            # CasADi optimizer
            opti = Opti()
            
            # Optimization variables
            nx = 3  # Number of states
            nu = 2  # Number of inputs
            x = opti.variable(nx, horizon + 1)  # States
            
            u = opti.variable(nu, horizon)  # Control inputs
            
            vr = opti.variable(1, horizon)  # Right wheel velocity
            vl = opti.variable(1, horizon)  # Left wheel velocity
            
            a = opti.variable(1, horizon)  # Linear acceleration
            alpha = opti.variable(1, horizon)  # Angular acceleration
            
            # Parameters
            x0 = opti.parameter(nx,1)  # Initial state
            u_prev = opti.parameter(nu,1)  # Previous control input

            # Define state and control model
            f = lambda x,u: vertcat(u[0]*cos(x[2]), u[0]*sin(x[2]), u[1]) # dx/dt = f(x,u)

            # Define objective function
            objective = 0
            for k in range(horizon): 
                
                # Tracking error cost
                objective += mtimes([(x[:, k + 1] - ref).T, Q, (x[:, k + 1] - ref)]) + mtimes([u[:, k].T, R, u[:, k]])

                    
                # Runge-Kutta 4 integration
                k1 = f(x[:,k],          u[:,k])
                k2 = f(x[:,k]+T_s/2*k1, u[:,k])
                k3 = f(x[:,k]+T_s/2*k2, u[:,k])
                k4 = f(x[:,k]+T_s*k3,   u[:,k])
                x_next = x[:,k] + T_s/6*(k1+2*k2+2*k3+k4) 
                opti.subject_to(x[:,k+1]==x_next) 
                
            # Minimize the objective
            opti.minimize(objective)

            # Control input constraints
            opti.subject_to(opti.bounded(-v_max, u[0,:], v_max)) # linear speed is limited
            opti.subject_to(opti.bounded(-omega_max, u[1,:], omega_max)) # linear speed is limited


            opti.subject_to(x[:,0]==x0)  


            # Linear acceleration constraint
            a = (u[0,:] - u_prev[0]) / T_s
            opti.subject_to(opti.bounded(-a_max, a, a_max))
            
            # Angular acceleration constraint
            alpha = (u[1, :] - u_prev[1]) / T_s
            opti.subject_to(opti.bounded(-alpha_max , alpha, alpha_max ))

            # Enforce relationships between u, vr, and vl
            opti.subject_to(u[0,:] == 0.5*(vl + vr))
            opti.subject_to(u[1,:] == (vr - vl)/b)


            # ---- make the solver silent ------ to see the time of computation, iterations... remove or change to True
            opts = {}
            another_opts = {}
            opts['verbose'] = False
            opts['print_time'] = False
            another_opts['print_level'] = 0

            # ---- solve NLP  ------
            opti.opts = {"ipopt.tol":1e-10, "expand":True}
            opti.solver("ipopt", opts,another_opts) # set numerical backend

            self.controller = opti.to_function("controller", [u_prev, x0], [u[:,0]], ["u_prev", "x0"], ["u_opt"]) 

        # Call the controller to get the optimal control inputs
        return self.controller(self.u_prev, robot_states)
    


if __name__ == '__main__':
    node_name = "controller_node"
    rospy.init_node(node_name)
    RobotController()
    rospy.spin()



