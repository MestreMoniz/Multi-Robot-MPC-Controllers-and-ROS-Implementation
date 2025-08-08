#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import casadi
from casadi import *
import math
from message_filters import Subscriber, ApproximateTimeSynchronizer



class DualRobotController:

    def __init__(self):

       # Topics for Jetbot 0
        pub_topic_name0 = "/jetbot0/diff_drive_controller/cmd_vel"
        sub_topic_name0 = "/jetbot0/diff_drive_controller/odom"

        # Topics for Jetbot 1
        pub_topic_name1 = "/jetbot1/diff_drive_controller/cmd_vel"
        sub_topic_name1 = "/jetbot1/diff_drive_controller/odom"

        # Publishers
        self.velocity_pub0 = rospy.Publisher(pub_topic_name0, Twist, queue_size=10)
        self.velocity_pub1 = rospy.Publisher(pub_topic_name1, Twist, queue_size=10)

        # Subscribers using message_filters
        self.odom_sub0 = Subscriber(sub_topic_name0, Odometry)
        self.odom_sub1 = Subscriber(sub_topic_name1, Odometry)

        # Synchronizing the callbacks
        self.sync = ApproximateTimeSynchronizer(
            [self.odom_sub0, self.odom_sub1], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.odometry_callback)

        
        # Command message
        self.velocity_msg0 = Twist()
        self.velocity_msg1 = Twist()
    
        
        #controllerMovie
        self.controller = None
        self.u_prev_1 = vertcat(0,0)
        self.u_prev_2 = vertcat(0,0)
        self.ref1 = vertcat(0,0)
        self.ref2 = vertcat(0,0)
        self.car_pos = vertcat(3,3)
        self.index = 0
        


    def odometry_callback(self, odom0, odom1):
         
         
        #Synchronized callback that processes odometry messages from both robots.
        

        # Extract position and orientation from Odometry message
        x0 = odom0.pose.pose.position.x
        y0 = odom0.pose.pose.position.y
        orientation_q0 = odom0.pose.pose.orientation

        x1 = odom1.pose.pose.position.x
        y1 = odom1.pose.pose.position.y
        orientation_q1 = odom1.pose.pose.orientation

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        quaternion0 = [orientation_q0.x, orientation_q0.y, orientation_q0.z, orientation_q0.w]
        _, _, theta0 = tf.transformations.euler_from_quaternion(quaternion0)
        # Log the extracted values
        rospy.loginfo("ROobo 1 - Position: x = %.2f, y = %.2f, Theta = %.2f", x0, y0, theta0)


        # Convert quaternion to Euler angles (roll, pitch, yaw)
        quaternion1 = [orientation_q1.x, orientation_q1.y, orientation_q1.z, orientation_q1.w]
        _, _, theta1 = tf.transformations.euler_from_quaternion(quaternion1)
        # Log the extracted values
        rospy.loginfo("Robo 2- Position: x = %.2f, y = %.2f, Theta = %.2f", x1, y1, theta1)


        # State vector as a column vector (6x1)
        state_vector = np.array([[x0], [y0], [theta0], [x1], [y1], [theta1]])
        
        
        # Call the controllerMovie function
        control_output = self.controllerMovie(state_vector)
        self.u_prev_1 = control_output[0:2,0]
        self.u_prev_2 = control_output[2:4,0]
        
        
        
        # Assign linear and angular speeds to the Twist message
        self.velocity_msg0.linear.x = control_output[0, 0]  # Linear speed
        self.velocity_msg0.angular.z = control_output[1, 0]  # Angular speed
        self.velocity_msg0.linear.x = control_output[2, 0]  # Linear speed
        self.velocity_msg0.angular.z = control_output[3, 0]  # Angular speed

        # Log the control actions without non-ASCII characters
        rospy.loginfo("Acoes de controlo para o Robo 1: Velocidade Linear={:.2f} m/s, Velocidade Angular={:.2f} rad/s".format(control_output[0, 0], control_output[1, 0]))
        rospy.loginfo("Acoes de controlo para o Robo 2: Velocidade Linear={:.2f} m/s, Velocidade Angular={:.2f} rad/s".format(control_output[2, 0], control_output[3, 0]))


        # Publish the command
        self.velocity_pub0.publish(self.velocity_msg)
        self.velocity_pub1.publish(self.velocity_msg)

    

    


    def controllerMovie(self, robot_states):
        
        if self.controller == None:
             
            
            b = 0.22 
            vmax = 1 # m/s
            amax = 3 # m/s2
            alfamax = amax * (2/b) #rad/s



            Th = 1.2 # horizon time #Aumentei o Th porque agora com runge kutta 4  possivel ter previses mais acertadas 
            Nh = 6 # number of control intervals
            dt = Th/Nh # length of a control interval
            nx = 3 # number of states
            nu = 2 # number of inputs
            ny = 2 # number of outputs


            Ts = 0.2
            T = 80
            numberIterations = int(round(T/Ts))
                                                            
            time_beyond_mpc= dt*Nh*0.5 #s 
            iterations_beyond_mpc = int(round(time_beyond_mpc/dt))
            
            #car beeing recorded parameters
            car_speed_x = 0.1 
            car_speed_y = 0.1

            car_x0 = self.car_pos[0]
            car_y0 = self.car_pos[1]
            
            self.car_pos[0,:] = DM(np.array(range(0, Nh + numberIterations + iterations_beyond_mpc ))*dt*car_speed_x + car_x0 )
            self.car_pos[1,:] = DM(np.array(range(0, Nh + numberIterations + iterations_beyond_mpc ))*dt*car_speed_y + car_y0)
            
            robot1_states = robot_states[0:2]
            robot2_states = robot_states[2:4]
            
            C = DM([[1, 0, 0],[0, 1, 0]])
            
            tau = self.desfasamento_obtencao(robot1_states, robot2_states, vertcat(car_x0, car_y0), C)
            
            self.ref1[0,:] = self.car_pos[0,:] + 2*cos(range(len(self.car_pos[0,:]))*dt*2*pi/40 + tau)
            self.ref1[1,:] = self.car_pos[1,:] + 2*sin(range(len(self.car_pos[0,:]))*dt*2*pi/40 + tau)
            
            self.ref2[0,:] = self.car_pos[0,:] + 2*cos(range(len(self.car_pos[0,:]))*dt*2*pi/40 + tau + pi)
            self.ref2[1,:] = self.car_pos[1,:] + 2*sin(range(len(self.car_pos[0,:]))*dt*2*pi/40 + tau + pi)
            
            
            
            opti = Opti() # Optimization problem

            # ---- decision variables ---------
            X = opti.variable(nx,Nh+1) # state trajectory
            state_x   = X[0,:] # position x
            state_y = X[1,:] # position y
            state_theta   = X[2,:] # orientation

            U = opti.variable(nu,Nh)   # control trajectory (linear and angular speed)
            input_linear_vel = U[0,:]
            input_angular_vel = U[1,:]

            Vl = opti.variable(1,Nh) 
            Vr = opti.variable(1,Nh) 

            Ref = opti.variable(ny,Nh+1)
            ref_x = Ref[0,:]
            ref_y = Ref[1,:]


            OldU = opti.variable(nu,1)

            PosVehicle = opti.variable(2,Nh+1)
            Pos_x_Vehicle = PosVehicle[0,:]
            Pos_y_Vehicle = PosVehicle[1,:]


            X0_parameter = opti.parameter(nx,1)
            OldU_parameter = opti.parameter(nu,1)
            Ref_x_parameter = opti.parameter(1,Nh+1)
            Ref_y_parameter = opti.parameter(1,Nh+1)
            Pos_x_Vehicle_parameter = opti.parameter(1,Nh+1)
            Pos_y_Vehicle_parameter = opti.parameter(1,Nh+1)

            # ---- objective          ---------
        
            Q = 7
            R = 1

            Y = mtimes(C,X) # para multiplicar matrizes
            difyr = Y - Ref

         

            # ---- dynamic constraints --------
            f = lambda x,u: vertcat(u[0]*cos(x[2]), u[0]*sin(x[2]), u[1]) # dx/dt = f(x,u)
            objective = 0
            for k in range(Nh): # loop over control intervals   

               relativePos = Y[:,k] - PosVehicle[:,k]
               
               crossPro = self.MX_skew(vertcat(f(X[:,k], U[:,k])[0:2],0))*vertcat(relativePos,0)

               objective = objective + sumsqr(U[:,k])*R + sumsqr(difyr[:,k])*Q + crossPro[3]

               # Runge-Kutta 4 integration
               k1 = f(X[:,k],         U[:,k])
               k2 = f(X[:,k]+dt/2*k1, U[:,k])
               k3 = f(X[:,k]+dt/2*k2, U[:,k])
               k4 = f(X[:,k]+dt*k3,   U[:,k])
               x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
               opti.subject_to(X[:,k+1]==x_next) # close the gaps

            objective = objective + sumsqr(difyr[:,k+1])*Q
            opti.minimize(objective)

            # ---- path constraints -----------
            #assume-se que ele so anda no sentido anti_horrio
            opti.subject_to(opti.bounded(0,input_linear_vel,vmax)) # linear speed is limited
            ######################################################################################################

            # ---- boundary conditions --------
            
            opti.subject_to(state_x[0]==X0_parameter[0])   
            opti.subject_to(state_y[0]==X0_parameter[1]) 
            opti.subject_to(state_theta[0]==X0_parameter[2])  

            opti.subject_to(ref_x[:]==Ref_x_parameter) 
            opti.subject_to(ref_y[:]==Ref_y_parameter) 

            opti.subject_to(OldU[0]==OldU_parameter[0])  
            opti.subject_to(OldU[1]==OldU_parameter[1])

            opti.subject_to(Pos_x_Vehicle[:]==Pos_x_Vehicle_parameter)
            opti.subject_to(Pos_y_Vehicle[:]==Pos_y_Vehicle_parameter)



            # ---- misc. constraints  ----------
            Us = horzcat(OldU,U)
            UsDiff = casadi.diff(Us,1,1)
            opti.subject_to(opti.bounded(-amax*dt,UsDiff[0,:],amax*dt)) # linear speed is limited
            opti.subject_to(opti.bounded(-alfamax*dt,UsDiff[1,:],alfamax*dt)) # linear speed is limited
            opti.subject_to(input_linear_vel == 0.5*(Vl + Vr))
            opti.subject_to(input_linear_vel == (Vr - Vl)/b)



            # ---- initial values for solver ---
            opti.set_initial(input_linear_vel, 1) #este valor tambm deveria dar para mudar e ser igual ao oldu_parameter mas tentei e no deu
            opti.set_initial(input_angular_vel, 1)

            # ---- make the solver silent ------ to see the time of computation, iterations... remove or change to True
            opts = {}
            another_opts = {}
            opts['verbose'] = False
            opts['print_time'] = False
            another_opts['print_level'] = 0

            # ---- solve NLP  ------
            opti.opts = {"ipopt.tol":1e-10, "expand":True}
            opti.solver("ipopt", opts,another_opts) # set numerical backend


            controller = opti.to_function("controller", [OldU_parameter, X0_parameter, Ref_x_parameter, Ref_y_parameter, Pos_x_Vehicle_parameter, Pos_y_Vehicle_parameter], [U[:,0]], ["u_prev", "x0", "ref_x", "ref_y", "pos_x_vehicle", "pos_y_vehicle"], ["u_opt"])
  


        
        if norm_2(mtimes(C, robot_states[0:2])- self.ref1[:,self.index]) > 3: 
          
            k1 = self.index + iterations_beyond_mpc
            
            
        else:
            k1 = self.index
        
       
        if norm_2(mtimes(C, robot_states[2:4])- self.ref2[:,self.index]) > 3: 
       
            k2 = self.index + iterations_beyond_mpc
            
        else:
            k2 = self.index
      
        
        aux1 = np.array(range(k1,k1 + Nh +1)) 
        aux2 = np.array(range(k2,k2 + Nh +1)) 
        
            
           
        u1 = self.controller(self.u_prev_1, robot_states[0:2], self.ref1[0,aux1], self.ref1[1,aux1], self.car_pos[0,aux1[:-1]], self.car_pos[1,aux1[:-1]])
        u2 = self.controller(self.u_prev_2, robot_states[2:4], self.ref2[0,aux2], self.ref2[1,aux2], self.car_pos[0,aux2[:-1]], self.car_pos[1,aux2[:-1]])
        
        self.index += 1
            
        return vertcat(u1, u2)
    
    
    
    def desfasamento_obtencao(x01,x02, pos_i, C):
    
        tau = math.atan((x01[1]-x02[1])/(x01[0]-x02[0]))

        aux1 = norm_2(mtimes(C,x01) - 2*vertcat(cos(tau), sin(tau)) - pos_i)
        aux2 = norm_2(mtimes(C,x02) - 2*vertcat(cos(tau+pi), sin(tau+pi)) - pos_i)

        aux3 = norm_2(mtimes(C,x01) - 2*vertcat(cos(tau+pi), sin(tau+pi)) - pos_i)
        aux4 = norm_2(mtimes(C,x02) - 2*vertcat(cos(tau), sin(tau)) - pos_i)

        if aux1 + aux2 < aux3 + aux4:
            return tau
        else :
            return tau+pi
        
    def MX_skew(x):
        # Obtain skew-symmetric matrix from vector

        n = MX.size(x)[0]
        if n == 3:
            X = vertcat(   horzcat(0,      -x[(2)],    x[(1)]), horzcat(x[(2)],    0,      -x[(0)]),  horzcat(-x[(1)],    x[(0)],    0)     );
        elif n == 1:
            X = vertcat(   horzcat(0,      -x[(0)]), horzcat(x[(0)],    0)    );
        else:
        # Use rospy.logerr for error handling
            rospy.logerr("SKEW function not implemented for input dimensions other than 1 or 3 (i.e., so(2) and so(3)).")
            return None  # Or raise an error depending on your preference

        return X

if __name__ == '__main__':
    node_name = "dual_robot_controller"
    rospy.init_node(node_name)
    DualRobotController()
    rospy.spin()



