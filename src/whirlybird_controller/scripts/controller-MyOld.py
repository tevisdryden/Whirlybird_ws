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

        # DD-fsg-tdryden-Downloads-Aaron-
        self.D_theta_p = 0.0
        self.D_phi_p = 0.0
        self.D_psi_p = 0.0
        self.theta_e_p = 0.0
        self.phi_e_p = 0.0
        self.psi_e_p = 0.0

        # Int
        self.I_theta_p = 0.0
        self.I_phi_p = 0.0
        self.I_psi_p = 0.0

        # Integrators
        self.ki_theta = 0.4
        self.ki_phi = 0.5
        self.ki_psi = 0.0

        # self.ki_theta = .5
        # self.ki_phi = 0.5
        # self.ki_psi = 0.5

    	# k values
    	# self.kd = 3.375;
    	# self.kp = 6.565;
        self.kd = 1.929 / .707 * 1.2;
    	self.kp = 2.143;
        self.kpInner = .252756
        self.kdInner = .048736

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

    	# Calculate theta dot
        sig = .05
        beta = (2 * sig - dt)/(2 * sig + dt)
    	self.D_theta = (theta - self.prev_theta)/dt;
        theta_e = self.theta_r - theta;
        #D_theta_ = beta * self.D_theta_p + (1 - beta)/dt * (theta_e - self.theta_e_p);
        I_theta_ = self.I_theta_p + dt / 2 * (theta_e + self.theta_e_p)
        thing = abs((theta_e - self.theta_e_p)/dt)
        if (thing > .5):
            I_theta = 0;
        self.I_theta_p = I_theta_;
        print(thing, " ",theta_e," ", I_theta_)
        self.theta_e_p = theta_e;
    	self.prev_theta = theta;
        self.D_theta_p = self.D_theta_;
        self.D_phi_ = (phi - self.prev_phi)/dt;

        self.D_psi_ = (psi - self.prev_psi)/dt;
        psi_e = self.psi_r - psi;
        #D_psi_ = beta * self.D_psi_p + (1 - beta)/dt * (psi_e - self.psi_e_p);
        I_psi_ = self.I_psi_p + dt / 2 * (psi_e - self.psi_e_p)
        thing2 = abs((psi_e - self.psi_e_p)/dt)
        if (thing2 > .5):
            I_psi_ = 0;
        self.I_psi_p = I_psi_;
        self.psi_e_p = psi_e;
        self.D_psi_p = self.D_psi_;

        ##################################
        # Implement your controller here

    	Fe = (m1 * l1 - m2 * l2) * g / l1 * np.cos(theta);

        b = (l1 * Fe/(m1 * l1**2 + m2 * l2**2 + Jz))
        kpOuter = (2.2 / 3)**2 / b
        kdOuter = 2.2 / 3 * 2 * .707 / b

    	FTild = self.kp * (self.theta_r - theta) - self.kd * self.D_theta + I_theta_ * self.ki_theta;

        phiC = (self.psi_r - psi) * kpOuter - kdOuter * self.D_psi_;

        phi_e = phiC - phi;
        I_phi_ = self.I_phi_p + dt / 2 * (phi_e - self.phi_e_p);
        self.I_phi_p = I_phi_; + I_theta_ * self.ki_theta
        self.phi_e_p = phi_e;
        #D_phi_ = beta * self.D_phi_p + (1 - beta)/dt * (phi_e - self.phi_e_p);
        self.D_phi_p = self.D_phi_;

        tau = (phiC - phi) * self.kpInner - self.kdInner * self.D_phi_  + I_psi_ * self.ki_phi;
        self.D_psi_ = (psi - self.prev_psi)/dt;
        self.prev_phi = phi;
        self.prev_psi = psi;

    	F = Fe + FTild;

    	left_force = F/2 + tau/(2*d);

    	right_force = F/2 - tau/(2*d);



        ########################self.D_psi_ = (psi - self.prev_psi)/dt;##########

        # Scale Output
        l_out = left_force/km
        if(l_out < 0):
            l_out = 0
        elif(l_out > .7):
            l_out = .7

        r_out = right_force/km
        if(r_out < 0):
            r_out = 0
        elif(r_out > .7):
            r_out = .7

        # Pack up and send command
        command = Command()
        command.left_motor = l_out
        command.right_motor = r_out
        self.command_pub_.publish(command)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    try:
        controller = Controller()
    except:
        rospy.ROSInterruptException
    pass
