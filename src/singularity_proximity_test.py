#!/usr/bin/env python3

import panda_py
import panda_py.controllers


# initialize robot, set velo controller and configure settings for collision behaviour
robot = panda_py.Panda("172.22.2.4")
robot.move_to_start(speed_factor = 0.1)

robot.move_to_joint_position([-0.10144671644453417, 0.11944516800683841, 0.10287828489352614, -2.197427476180227, -0.015906027694575792, 2.3155323374843864, 1.6445289437659227], speed_factor = 0.05)
robot.move_to_joint_position([-0.2893883252342542, 1.2422407621249818, 0.974523238649323, -0.5878482881512727, -0.8959023066885152, 1.59118953233295, 1.8205427699647845], speed_factor = 0.05)