#!/usr/bin/env python
# license removed for brevity


# This file is a basic structure to write a controller that
# communicates with ROS. It will be the students responsibility
# tune the gains and fill in the missing information

# As an example this file contains PID gains, but other
# controllers use different types of gains so the class
# will need to be modified to accomodate those changes

import rospy
import time
import numpy as np
from numpy import matrix, linalg
from math import sin, cos
import control
from control import place
from whirlybird_msgs.msg import Command
from whirlybird_msgs.msg import Whirlybird
from std_msgs.msg import Float32


class Controller():

    def __init__(self):

        # get parameters
        try:
            param_namespace = '/whirlybird'
            self.param = rospy.get_param(param_namespace)
        except KeyError:
            rospy.logfatal('Parameters not set in ~/whirlybird namespace')
            rospy.signal_shutdown('Parameters not set')

        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']


        # Roll Gains
        self.P_phi_ = 0.0
        self.I_phi_ = 0.0
        self.D_phi_ = 0.0
        self.Int_phi = 0.0
        self.prev_phi = 0.0

        # Pitch Gains
        self.theta_r = 0.0
        self.P_theta_ = 0.0
        self.I_theta_ = 0.0
        self.D_theta_ = 0.0
        self.prev_theta = 0.0
        self.Int_theta = 0.0

        # Yaw Gains
        self.psi_r = 0.0
        self.P_psi_ = 0.0
        self.I_psi_ = 0.0
        self.D_psi_ = 0.0
        self.prev_psi = 0.0
        self.Int_psi = 0.0

        # k values
        # self.kd_theta = 3.375
        # self.kp_theta = 6.565
        self.integral_prev_theta = 0
        self.integral_prev_psi = 0
        self.integral_prev_phi = 0

        #Calculate Gains
            #psi is yaw, phi is roll, theta is pitch

        self.tr_theta =   .8
        self.tr_phi   =  .2
        self.tr_psi   =   self.tr_phi*5.5
        self.kd_theta =  ((2.2 / self.tr_theta)*2*.707) / 1.152 #1.929
        self.kp_theta =  (2.2 / self.tr_theta)**2 / 1.152  #2.143
        self.kd_phi   =  ((2.2 / self.tr_phi)*2*1.1) * Jx #0.048736
        self.kp_phi   =  (2.2 / self.tr_phi)**2 * Jx #0.252756
        self.ki_phi   = 0.0
        self.ki_psi   = .4
        self.ki_theta = 1.5

        b = (l1*(m1*l1 - m2*l2) * (g/l1))/((m1*(l1**2))+(m2*(l2**2))+Jz)
        self.kd_psi   = ((2.2 / self.tr_psi)*2*2) / b
        self.kp_psi   = (2.2 / self.tr_psi)**2 / b

        print("Rise Time Theta ", self.tr_theta)
        print("Kp Theta ", self.kp_theta)
        print("Kd Theta ", self.kd_theta)
        print("Ki Theta ", self.ki_theta)
        print("Rise Time Phi ", self.tr_phi)
        print("Kp Phi ", self.kp_phi)
        print("Kd Phi ", self.kd_phi)
        print("Ki Phi ", self.ki_phi)
        print("Rise Time Psi ", self.tr_psi)
        print("Kp Psi ", self.kp_psi)
        print("Kd Psi ", self.kd_psi)
        print("Ki Psi ", self.ki_psi)




        self.prev_time = rospy.Time.now()

        self.Fe = 0.0 #Note this is not the correct value for Fe, you will have to find that yourself

        self.command_sub_ = rospy.Subscriber('whirlybird', Whirlybird, self.whirlybirdCallback, queue_size=5)
        self.psi_r_sub_ = rospy.Subscriber('psi_r', Float32, self.psiRCallback, queue_size=5)
        self.theta_r_sub_ = rospy.Subscriber('theta_r', Float32, self.thetaRCallback, queue_size=5)
        self.command_pub_ = rospy.Publisher('command', Command, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def thetaRCallback(self, msg):
        self.theta_r = msg.data


    def psiRCallback(self, msg):
        self.psi_r = msg.data


    def whirlybirdCallback(self, msg):
        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']

        phi = msg.roll
        theta = msg.pitch
        psi = msg.yaw

        # Calculate dt (This is variable)
        now = rospy.Time.now()
        dt = (now-self.prev_time).to_sec()
        self.prev_time = now

        self.D_theta = (theta - self.prev_theta)/dt
        self.prev_theta = theta

        self.D_psi = (psi - self.prev_psi)/dt
        self.prev_psi = psi

        self.D_phi = (phi - self.prev_phi)/dt
        self.prev_phi = phi

        Fe = (m1*l1 - m2*l2) * (g/l1)

        ##########################################
        # Implement your SS controller here
        val = l1 * Fe / (m1 * l1**2 + m2 * l2**2 + Jz)
        A_lat = matrix([[0,0,1,0],[0,0,0,1],[0,0,0,0],[val,0,0,0]])
        B_lat = matrix([[0],[0],[1 / Jx],[0]])
        C_lat = matrix([[1,0,0,0],[0,1,0,0]])

        theta_e = 0;

        val2 = (m1 * l1 - m2 * l2) * g * sin(theta_e) / (m1 * l1**2 + m2 * l2**2 + Jy)
        A_lon = matrix([[0,1],[val2,0]])
        val3 = l1 / (m1 * l1**2 + m2 * l2**2 + Jy)
        B_lon = matrix([[0],[val3]])
        C_lon = matrix([[1,0]])

        zeta_psi = .707;
        zeta_phi = .707;
        wn_psi = 2.2/self.tr_psi
        wn_phi = 2.2/self.tr_phi
        desired_poles = np.roots(np.convolve([1, 2*zeta_phi*wn_phi,wn_phi**2],[1, 2*zeta_psi*wn_psi,wn_psi**2]))
        K_lat = place(A_lat,B_lat,desired_poles)
        Kr_lat = -1/(C_lat[1]*linalg.inv(A_lat-B_lat*K_lat)*B_lat)
        x_lat = matrix([[phi],[psi],[self.D_phi],[self.D_psi]])

        zeta_theta = .707
        wn_theta = 2.2/self.tr_theta
        desired_poles_lon = np.roots([1, 2*zeta_theta*wn_theta,wn_theta**2])
        K_lon = place(A_lon,B_lon,desired_poles_lon)
        Kr_lon = -1/(C_lon*linalg.inv(A_lon-B_lon*K_lon)*B_lon)
        x_lon = matrix([[theta],[self.D_theta]])

        Ftil = -K_lon*x_lon + Kr_lon*self.theta_r
        F = Fe + Ftil

        tau = -K_lat*x_lat + Kr_lat*self.psi_r

        left_force = F/2 + tau/(2*d)
        right_force = F/2 - tau/(2*d)
        # ##########################################
        # # Implement your controller here
        # sigma = .05
        #
        # self.D_theta = (theta - self.prev_theta)/dt
        # Fe = (m1*l1 - m2*l2) * (g/l1) * np.cos(theta)
        #
        # error_theta = self.theta_r-theta
        # error_prev_theta = self.theta_r-self.prev_theta
        # integral_theta = ((error_theta+error_prev_theta)/2.0)*dt + self.integral_prev_theta
        # if abs((error_theta-error_prev_theta)/dt) > 1:
        #     integral_theta = 0
        #
        # Ftil = (self.kp_theta * (self.theta_r - theta)) - (self.kd_theta*self.D_theta) + (self.ki_theta * integral_theta)
        #
        # self.integral_prev_theta = integral_theta
        # self.prev_theta = theta
        # F = Fe + Ftil
        #
        # error_psi = self.psi_r-psi
        # error_prev_psi = self.psi_r-self.prev_psi
        # self.D_psi = (psi - self.prev_psi)/dt
        # integral_psi = ((error_psi+error_prev_psi)/2.0)*dt + self.integral_prev_psi
        # if abs((error_psi - error_prev_psi)/dt) > 1:
        #     integral_psi = 0
        #
        # phi_r = (self.kp_psi * (self.psi_r - psi)) - (self.kd_psi*self.D_psi) + (self.ki_psi*integral_psi)
        #
        # self.integral_prev_psi = integral_psi
        # self.prev_psi = psi
        #
        # error_phi = phi_r-phi
        # error_prev_phi = phi_r-self.prev_phi
        # self.D_phi = (phi - self.prev_phi)/dt
        # integral_phi = ((error_phi+error_prev_phi)/2.0)*dt + self.integral_prev_phi
        # if abs((error_phi-error_prev_phi)/dt) > .5:
        #     integral_phi = 0
        #
        # tau = (self.kp_phi * (phi_r - phi)) - (self.kd_phi*self.D_phi) + (self.ki_phi*integral_phi)
        #
        # self.integral_prev_phi = integral_phi
        # self.prev_phi = phi
        #
        # left_force = F/2 + tau/(2*d)
        # right_force = F/2 - tau/(2*d)
        #
        # ###########################################

        # Scale Output
        l_out = left_force/km
        if(l_out < 0):
            l_out = 0
        elif(l_out > 0.6):
            l_out = 0.6

        r_out = right_force/km
        if(r_out < 0):
            r_out = 0
        elif(r_out > 0.6):
            r_out = 0.6

        # Pack up and send command
        command = Command()
        command.left_motor = l_out
        command.right_motor = r_out
        self.command_pub_.publish(command)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    controller = Controller()
    # try:
    #
    # except:
    #     rospy.ROSInterruptException
    # pass
