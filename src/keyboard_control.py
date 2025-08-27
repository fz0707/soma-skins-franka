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

# All raw data publishers
pub_q   = rospy.Publisher("/angle",     Float64MultiArray, queue_size=10)
pub_dq  = rospy.Publisher("/angle_vel", Float64MultiArray, queue_size=10)
pub_tau = rospy.Publisher("/torque",    Float64MultiArray, queue_size=10)
pub_angAcc = rospy.Publisher("/angle_acc",    Float64MultiArray, queue_size=10)
pub_eepose= rospy.Publisher("/ee_pose", Float64MultiArray, queue_size=10)
pub_man = rospy.Publisher("/manipulability", Float64MultiArray, queue_size=10)
pub_eeVelo = rospy.Publisher("/ee_velocity", Float64MultiArray, queue_size=10)

# All scaled data publishers
pub_q_scaled   = rospy.Publisher("/angle_scaled",     Float64MultiArray, queue_size=10)
pub_dq_scaled  = rospy.Publisher("/angle_vel_scaled", Float64MultiArray, queue_size=10)
pub_tau_scaled = rospy.Publisher("/torque_scaled",    Float64MultiArray, queue_size=10)
pub_angAcc_scaled = rospy.Publisher("/angle_acc_scaled",    Float64MultiArray, queue_size=10)
pub_eepose_scaled = rospy.Publisher("/ee_pose_scaled", Float64MultiArray, queue_size=10)
pub_man_scaled = rospy.Publisher("/manipulability_scaled", Float64MultiArray, queue_size=10)
pub_eeVelo_scaled = rospy.Publisher("/ee_velocity_scaled", Float64MultiArray, queue_size=10)


def publishInfo(currState):

    #get cartesian coord of the end effector
    eePose = getEndEffPose(currState)
    
    #Yoshikawa's manipulability to detect singularities
    w, eeVelo = computeManipulabilityandVelocity(currState)

    pub_q.publish(Float64MultiArray(data=list(currState.q)))
    pub_dq.publish(Float64MultiArray(data=list(currState.dq)))
    pub_tau.publish(Float64MultiArray(data=list(currState.tau_ext_hat_filtered)))
    pub_angAcc.publish(Float64MultiArray(data=list(currState.ddq_d)))
    pub_eepose.publish(Float64MultiArray(data=eePose))
    pub_man.publish(Float64MultiArray(data = [w]))
    pub_eeVelo.publish(Float64MultiArray(data = eeVelo))


def computeManipulabilityandVelocity(currState):

    J_body = panda_rtb.jacobe(currState.q) #compute body jacobian
    w = np.sqrt(np.linalg.det(J_body @ J_body.T)) # compute manipulability using Jacobian

    eeVelo = np.dot(J_body, currState.dq) # compute end eff velocity using Jacobian (6x1 : linear velo x, y, z and angular velo x, y and z)

    return w, eeVelo


def getEndEffPose(currState):

    eePosition = robot.get_position()
    eeOrientation = robot.get_orientation()
    eePose = list(eePosition) + list(eeOrientation)
    
    return eePose



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
    rate = rospy.Rate(10) #10Hz

    while not rospy.is_shutdown():

        currState = robot.get_state()
        publishInfo(currState)

        sub = rospy.Subscriber('/cmd_vel', Twist, send_command)

        rate.sleep()
