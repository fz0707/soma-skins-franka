#!/usr/bin/env python3

import rospy
import panda_py
import panda_py.controllers
import numpy as np


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




def calcTrq():

    torques = np.array([0, 0, 0, 0, 0, 0, 0])
    return torques



if __name__ == "__main__":
    rospy.init_node("kinesthetic_control")
    rate = rospy.Rate(10) #10Hz
    
    while not rospy.is_shutdown():
        
        currState = robot.get_state()

        #send torques to control the robot
        torques = calcTrq()
        trqController.set_control(torques)

        rate.sleep()