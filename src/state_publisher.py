#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64MultiArray
import panda_py
import panda_py.controllers
import numpy as np
import roboticstoolbox as rtb


# toorbox model for panda
panda_rtb = rtb.models.Panda()

# initialize robot, set torque controller and configure settings for collision behaviour
robot = panda_py.Panda("172.22.2.3")
trqController = panda_py.controllers.AppliedTorque()
robot.start_controller(trqController)
robotSettings = robot.get_robot()
robotSettings.set_collision_behavior(lower_torque_thresholds_acceleration = [x / 15 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                            upper_torque_thresholds_acceleration = [x * 15 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                            lower_torque_thresholds_nominal = [x / 15 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                            upper_torque_thresholds_nominal = [x * 15 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                            lower_force_thresholds_acceleration = [x / 15 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                            upper_force_thresholds_acceleration = [x * 15 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                            lower_force_thresholds_nominal = [x / 15 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                            upper_force_thresholds_nominal = [x * 15 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]])


# All raw data publishers
pub_q   = rospy.Publisher("/angle",     Float64MultiArray, queue_size=10)
pub_dq  = rospy.Publisher("/angle_vel", Float64MultiArray, queue_size=10)
pub_tau = rospy.Publisher("/torque",    Float64MultiArray, queue_size=10)
pub_angAcc = rospy.Publisher("/angle_acc",    Float64MultiArray, queue_size=10)
pub_eepose= rospy.Publisher("/eepose", Float64MultiArray, queue_size=10)
pub_man = rospy.Publisher("/manipulability", Float64MultiArray, queue_size=10)

# All scaled data publishers
pub_q_scaled   = rospy.Publisher("/angle_scaled",     Float64MultiArray, queue_size=10)
pub_dq_scaled  = rospy.Publisher("/angle_vel_scaled", Float64MultiArray, queue_size=10)
pub_tau_scaled = rospy.Publisher("/torque_scaled",    Float64MultiArray, queue_size=10)
pub_angAcc_scaled = rospy.Publisher("/angle_acc_scaled",    Float64MultiArray, queue_size=10)
pub_eepose_scaled = rospy.Publisher("/eepose_scaled", Float64MultiArray, queue_size=10)
pub_man_scaled = rospy.Publisher("/manipulability_scaled", Float64MultiArray, queue_size=10)


def publishInfo(currState):

    #get cartesian coord of the end effector
    eePose = computeCartesianCoord(currState)
    
    #Yoshikawa's manipulability to detect singularities
    w = computeManipulability(currState)

    pub_q.publish(Float64MultiArray(data=list(currState.q)))
    pub_dq.publish(Float64MultiArray(data=list(currState.dq)))
    pub_tau.publish(Float64MultiArray(data=list(currState.tau_ext_hat_filtered)))
    pub_angAcc.publish(Float64MultiArray(data=list(currState.ddq_d)))
    pub_eepose.publish(Float64MultiArray(data=eePose))
    pub_man.publish(Float64MultiArray(data = [w]))




def calcTrq():

    torques = np.array([0, 0, 0, 0, 0, 0, 0])
    return torques


def computeManipulability(currState):

    J_body = panda_rtb.jacobe(currState.q) #compute body jacobian
    w = np.sqrt(np.linalg.det(J_body @ J_body.T))

    return w


def computeCartesianCoord(currState):

    eePosition = robot.get_position()
    eeOrientation = robot.get_orientation()
    eePose = list(eePosition) + list(eeOrientation)
    
    return eePose




if __name__ == "__main__":
    rospy.init_node("state_publisher")
    rate = rospy.Rate(10) #10Hz
    
    while not rospy.is_shutdown():
        
        currState = robot.get_state()
        publishInfo(currState)

        #send torques to control the robot
        torques = calcTrq()
        trqController.set_control(torques)

        rate.sleep()