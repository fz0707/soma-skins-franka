#!/usr/bin/env python3

import panda_py
import panda_py.controllers
import threading
from std_msgs.msg import String, Float64MultiArray
import rospy

rospy.init_node('joint_prox_test')

# all publishers
pub_q   = rospy.Publisher("/angle", Float64MultiArray, queue_size=10)

# initialize robot, set velo controller and configure settings for collision behaviour
robot = panda_py.Panda("172.22.2.3")
robot.move_to_start()

def execute_movt():

    robot.move_to_joint_position([-0.9032751608187692, -0.986809818851969, 0.6083256887051097, -2.721406814430684, 2.3609683564139736, 2.4484525568338684, -0.3142462457915147], speed_factor = 0.05)
    robot.move_to_joint_position([-1.3325060756457479, -1.7629062470553214, 0.6931213590053588, -3.0553792524170458, 2.200219855220393, 1.8567968375550377, 0.31172335727181605], speed_factor = 0.05) 


def pub_data():

    rate = rospy.Rate(10) #10 Hz

    while True:

        currState = robot.get_state()
        pub_q.publish((Float64MultiArray(data=list(currState.q))))
        rate.sleep()


if  __name__ == '__main__':


    pub_thread = threading.Thread(target=pub_data)
    pub_thread.start()

    execute_movt()


