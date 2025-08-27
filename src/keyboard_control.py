#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import roboticstoolbox as rtb
import numpy as np
import panda_py
import panda_py.controllers


# toolbox model for panda
panda_rtb = rtb.models.Panda()


# initialize robot, set velo controller and configure settings for collision behaviour
robot = panda_py.Panda("172.22.2.3")
veloController = panda_py.controllers.IntegratedVelocity()
robot.start_controller(veloController)


def send_command(msg):

    currState = robot.get_state()
    twist = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z])
    dq = compute_joint_velo(currState, twist)

    veloController.set_control(dq)
    print(f'velocity set to {dq}')


def compute_joint_velo(currState, twist, damping = 0.01):

    J_body = panda_rtb.jacobe(currState.q)

    # damped least squares pseudoinv
    J_body_T = J_body.T
    J_body_pinv = J_body_T @ np.linalg.inv(J_body @ J_body_T + damping**2 * np.eye(6))

    #compute joint velo
    dq = J_body_pinv @ twist

    return dq


if __name__ == "__main__":
    rospy.init_node('keyboard_control')

    sub = rospy.Subscriber('/cmd_vel', Twist, send_command)
    
    rospy.spin()