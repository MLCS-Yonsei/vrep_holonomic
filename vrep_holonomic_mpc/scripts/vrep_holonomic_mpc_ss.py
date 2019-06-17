#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import time
from std_srvs.srv import Empty
import numpy as np
from tf import TransformListener
from tf.transformations import euler_from_quaternion
from sys import path
try:
    casadi_path = rospy.get_param("/vrep_holonomic_mpc/casadi_path")
    path.append(casadi_path)
except:
    pass
from casadi import *


class MPC_controller:

    def __init__(self, cmd_vel_topic):

        rospy.init_node('MPC_controller', anonymous=True)
        time.sleep(1)

        self.curx = 0
        self.cury = 0
        self.curyaw = 0
        self.gx = 0
        self.gy = 0
        self.gyaw = 0
        self.use_odom = rospy.get_param("/vrep_holonomic_mpc/use_odom")
        self.target_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.targetCallback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallback)
        self.tfListener = TransformListener()

        #declare velocity publisher
        self.vel_msg = Twist()
        self.vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.setup_MPC()


    def spin(self, rate):
        self.rate = rospy.Rate(rate)
        rospy.spin()


    def pose_from_tf(self):

        position, quaternion = self.tfListener.lookupTransform(
            "/map",
            "/base_footprint",
            self.tfListener.getLatestCommonTime("/map", "/base_footprint")
        )
        self.curx = position[0]
        self.cury = position[1]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.curyaw = yaw


    def odomCallback(self, msg):

        if self.use_odom:
            self.curx = msg.pose.pose.position.x
            self.cury = msg.pose.pose.position.y
            quat = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            (roll, pitch, yaw) = euler_from_quaternion(quat)
            self.curyaw = yaw
        else:
            self.pose_from_tf()

        self.publish_vel()


    def targetCallback(self, msg):

        self.gx = msg.pose.position.x
        self.gy = msg.pose.position.y
        quat = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        (roll, pitch, yaw) = euler_from_quaternion(quat)
        self.gyaw = yaw

        print 'Target pose :', self.gx, self.gy, self.gyaw, '\n'


    def publish_vel(self):

        if not self.use_odom:
            self.pose_from_tf(self.tfListener) # Robot pose from /tf if use_odom is False

        if self.compute_err() < 0.05:
            vel = [0.0, 0.0, 0.0]
        else:
            vel = self.iterate_MPC(self.N)

        self.vel_msg.linear.x  = vel[0]
        self.vel_msg.linear.y  = vel[1]
        self.vel_msg.angular.z = vel[2]
        self.vel_pub.publish(self.vel_msg)


    def compute_err(self):
        yawdiff = self.gyaw - self.curyaw
        if yawdiff > math.pi:
            yawdiff -= 2.0*math.pi
        elif yawdiff < -math.pi:
            yawdiff += 2.0*math.pi
        return np.linalg.norm(
            (self.gx - self.curx, self.gy - self.cury, yawdiff)
        )


    def setup_MPC(self):

        self.T = T = 0.1 # Time horizon
        self.N = N = 2 # number of control intervals
        self.rob_diam = 0.5

        '''X_max = 3
        X_min = -X_max
        Y_max = 3
        Y_min = -Y_max
        omega_max = float('inf')
        omega_min= -omega_max
        v_max = float('inf')
        v_min = -v_max
        w_max = float('inf')
        w_min = -w_max'''

        vX_max = 2
        vX_min = -vX_max
        vY_max = 0.6
        vY_min = -vY_max
        omega_max = math.pi/4
        omega_min= -omega_max

        #setting up symbolic variables for states
        x = SX.sym('x')
        y = SX.sym('y')
        theta = SX.sym('theta')
        states = vertcat(x,y,theta)
        self.n_states = states.size1()

        #setting up symbolic variables for control inputs
        vX = SX.sym('vX')
        vY = SX.sym('vY')
        omega = SX.sym('omega')
        controls = vertcat(vX,vY,omega)
        self.n_controls = controls.size1()
        rhs = vertcat(vX*cos(theta)-vY*sin(theta),vX*sin(theta)+vY*cos(theta),omega) # system r.h.s

        self.f = Function('f',[states,controls],[rhs]) # nonlinear mapping function f(x,u), system equation
        U = SX.sym('U', self.n_controls, N) # Decision variables (controls), optimal control input as determined by optimizer
        P = SX.sym('P',self.n_states + self.n_states)
        # parameters (which include the initial and the reference/final desired state of the robot)

        X = SX.sym('X',self.n_states,(N+1))

        obj = 0; # Objective function
        g = [];  # constraints vector

        q1 = 2
        q2 = 2
        q3 = 4

        r1 = 4
        r2 = 4
        r3 = 8
        Q = horzcat([q1,0,0],[0,q2,0],[0,0,q3])
        R = horzcat([r1,0,0],[0,r2,0],[0,0,r3])

        # compute objective symbollically this is what the solver will minimize
        X[:,0] = P[0:3] # initial state
        for k in range(0, N):
            st = X[:,k]
            con = U[:,k]
            f_value = self.f(st,con)
            st_next = st+(T*f_value)
            X[:,k+1] = st_next

        self.ff = Function('ff',[U,P],[X])

        obj = 0; # Objective function
        g = [];  # constraints vector

        for k in range(0,N):
            st = X[:,k];  con = U[:,k]
            obj = obj+mtimes(mtimes((st-P[3:6]).T,Q),(st-P[3:6]))+ mtimes(mtimes(con.T,R),con) # calculate obj

        #compute constraints, constraints for each state x and y
        for k in range(0,N):   # box constraints due to the map margins
            g = vertcat(g, X[0,k], X[1,k])

        #setup variables for input contraints
        w_max = 7
        wheel_rad = 0.076
        a = 0.281
        b = 0.2355

        H = np.array(([1, -1, (-a-b)],
            [1, 1, (a+b)],
            [1, -1, (a+b)],
            [1, 1, (-a-b)]))

        for k in range(0, N):
            con = U[:,k]
            g = vertcat(g, mtimes(H,con))

        # make the decision variables one column vector, these alternate between
        # states
        OPT_variables = reshape(U,3*N,1)

        '''opts = {
            'ipopt.max_iter': 2000,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.acceptable_tol': 1e-8,
            'ipopt.acceptable_obj_change_tol': 1e-6
        }'''

        nlp_prob = {'f':obj, 'x':OPT_variables, 'g':g, 'p':P}
        self.solver = nlpsol('solver', 'ipopt', nlp_prob)

        lbg = np.zeros((6*N,1))
        ubg = np.zeros((6*N,1))

        # inequality constraints (state constraints)
        lbg [0:2*N] = -3  # lower bound of the states x and y
        ubg [0:2*N] = 3.5  # upper bound of the states x and y

        #g constraints max wheel velocity
        lbg[2*N:6*N] = -wheel_rad*w_max
        ubg[2*N:6*N] = wheel_rad*w_max

        lbx = np.zeros(3*N)
        ubx = np.zeros(3*N)

        # input constraints
        for k in range(0,N):
            lbx[3*k]   = vX_min
            lbx[3*k+1] = vY_min
            lbx[3*k+2] = omega_min
            ubx[3*k]   = vX_max
            ubx[3*k+1] = vY_max
            ubx[3*k+2] = omega_max

        self.args = {'lbg':lbg,'ubg':ubg,'lbx':lbx,'ubx':ubx}#, 'p':P, 'x0':P[0:3]}
        self.u0 = np.zeros((N, self.n_controls))


    def iterate_MPC(self, N):

        if not self.use_odom:
            self.pose_from_tf(self.tfListener) # Robot pose from /tf if use_odom is False

        x0 = np.array([[self.curx], [self.cury] , [self.curyaw]]);    # initial condition.
        xs = np.array([[self.gx], [self.gy], [self.gyaw]]); # Reference posture.
        #xs = np.array([[0], [0], [0]])


        self.args['p'] = vertcat(x0, xs) # set the values of the parameters vector
        self.par =  DM(self.args['p'])
        self.args['x0'] = reshape(self.u0.T,3*N,1) # initial value of the optimization variables, w0 is the optimization variable
        sol = self.solver(
            x0  = self.args['x0'],
            lbx = self.args['lbx'],
            ubx = self.args['ubx'],
            lbg = self.args['lbg'],
            ubg = self.args['ubg'],
            p   = self.par
        )

        u = reshape((sol['x']).T,self.n_controls,N).T
        u0 = vertcat(u[1:],u[-1])

        return u[0,0:]



if __name__ == '__main__':

    try:
        cmd_vel_topic='/cmd_vel'
        controller = MPC_controller(cmd_vel_topic)
        controller.spin(10)
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
