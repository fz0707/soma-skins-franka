#!/usr/bin/env python3

import panda_py
import panda_py.controllers
import threading
from std_msgs.msg import String, Float64MultiArray
import rospy
import numpy as np
import roboticstoolbox as rtb

rospy.init_node('simgularityprox_test')

# toorbox model for panda
panda_rtb = rtb.models.Panda()

# All publishers
pub_man = rospy.Publisher("/manipulability", Float64MultiArray, queue_size=10)

# initialize robot, set velo controller and configure settings for collision behaviour
robot = panda_py.Panda("172.22.2.3")
robot.move_to_start(speed_factor = 0.1)



def execute_movt():
    robot.move_to_joint_position([-0.10144671644453417, 0.11944516800683841, 0.10287828489352614, -2.197427476180227, -0.015906027694575792, 2.3155323374843864, 1.6445289437659227], speed_factor = 0.05)
    robot.move_to_joint_position([-0.2893883252342542, 1.2422407621249818, 0.974523238649323, -0.5878482881512727, -0.8959023066885152, 1.59118953233295, 1.8205427699647845], speed_factor = 0.05)



def computeManipulabilityandVelocity(currState):

    J_body = panda_rtb.jacobe(currState.q) #compute body jacobian
    w = np.sqrt(np.linalg.det(J_body @ J_body.T)) # compute manipulability using Jacobian

    eeVelo = np.dot(J_body, currState.dq) # compute end eff velocity using Jacobian (6x1 : linear velo x, y, z and angular velo x, y and z)

    return w, eeVelo



def pub_data():

    rate = rospy.Rate(10) #10 Hz

    while True:

        currState = robot.get_state()

        #Yoshikawa's manipulability to detect singularities
        w, eeVelo = computeManipulabilityandVelocity(currState)

        pub_man.publish(Float64MultiArray(data = [w]))
        
        rate.sleep()



if  __name__ == '__main__':


    pub_thread = threading.Thread(target=pub_data)
    pub_thread.start()

    execute_movt()


