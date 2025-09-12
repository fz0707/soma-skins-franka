from argparse import ArgumentParser
from time import sleep
from frankx import Affine, Robot, JointMotion, PathMotion, WaypointMotion, Waypoint, MotionData, Reaction, Measure, StopMotion, LinearRelativeMotion

def program():
  parser = ArgumentParser()
  parser.add_argument('--host', default='172.22.2.4', help='FCI IP of the robot')
  args = parser.parse_args()
  robot = Robot(args.host)
  robot.set_default_behavior()
  robot.recover_from_errors()
  robot.velocity_rel = 1.0

  DefaultTool = Affine(0, 0, 0, 0, 0, 1, 0)
  DefaultZone = 0
  dynamic_rel = 0.25
  robot.acceleration_rel = dynamic_rel
  robot.jerk_rel = dynamic_rel
  data = MotionData(dynamic_rel)
  data.velocity_rel = 0.1
  motion = JointMotion([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
  robot.move(motion, data)
  motion = WaypointMotion([
    Waypoint(Affine(0.54549, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.10292),
    # Waypoint(Affine(0.54811, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.13934, DefaultZone),
    # Waypoint(Affine(0.55072, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.13987, DefaultZone),
    # Waypoint(Affine(0.55333, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14041, DefaultZone),
    # Waypoint(Affine(0.55594, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14097, DefaultZone),
    # Waypoint(Affine(0.55855, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14153, DefaultZone),
    # Waypoint(Affine(0.56117, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.1421, DefaultZone),
    # Waypoint(Affine(0.56378, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14269, DefaultZone),
    # Waypoint(Affine(0.56639, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14328, DefaultZone),
    # Waypoint(Affine(0.569, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14389, DefaultZone),
    # Waypoint(Affine(0.57161, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.1445, DefaultZone),
    # Waypoint(Affine(0.57423, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14513, DefaultZone),
    # Waypoint(Affine(0.57684, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14578, DefaultZone),
    # Waypoint(Affine(0.57945, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14643, DefaultZone),
    # Waypoint(Affine(0.58206, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.1471, DefaultZone),
    # Waypoint(Affine(0.58468, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14778, DefaultZone),
    # Waypoint(Affine(0.58729, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14848, DefaultZone),
    # Waypoint(Affine(0.5899, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14919, DefaultZone),
    # Waypoint(Affine(0.59251, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.14991, DefaultZone),
    # Waypoint(Affine(0.59512, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15065, DefaultZone),
    # Waypoint(Affine(0.59774, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15141, DefaultZone),
    # Waypoint(Affine(0.60035, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15218, DefaultZone),
    # Waypoint(Affine(0.60296, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15297, DefaultZone),
    # Waypoint(Affine(0.60557, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15377, DefaultZone),
    # Waypoint(Affine(0.60819, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15459, DefaultZone),
    # Waypoint(Affine(0.6108, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15543, DefaultZone),
    # Waypoint(Affine(0.61341, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15629, DefaultZone),
    # Waypoint(Affine(0.61602, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15717, DefaultZone),
    # Waypoint(Affine(0.61863, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15807, DefaultZone),
    # Waypoint(Affine(0.62125, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15899, DefaultZone),
    # Waypoint(Affine(0.62386, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.15993, DefaultZone),
    # Waypoint(Affine(0.62647, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.16089, DefaultZone),
    # Waypoint(Affine(0.62908, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.16188, DefaultZone),
    # Waypoint(Affine(0.6317, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.16289, DefaultZone),
    # Waypoint(Affine(0.63431, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.16392, DefaultZone),
    # Waypoint(Affine(0.63692, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.16498, DefaultZone),
    # Waypoint(Affine(0.63953, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.16607, DefaultZone),
    # Waypoint(Affine(0.64214, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.16718, DefaultZone),
    # Waypoint(Affine(0.64476, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.16833, DefaultZone),
    # Waypoint(Affine(0.64737, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.1695, DefaultZone),
    # Waypoint(Affine(0.64998, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.1707, DefaultZone),
    # Waypoint(Affine(0.65259, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.17194, DefaultZone),
    # Waypoint(Affine(0.6552, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.17321, DefaultZone),
    # Waypoint(Affine(0.65782, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.17451, DefaultZone),
    # Waypoint(Affine(0.66043, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.17586, DefaultZone),
    # Waypoint(Affine(0.66304, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.17724, DefaultZone),
    # Waypoint(Affine(0.66565, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.17866, DefaultZone),
    # Waypoint(Affine(0.66827, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.18012, DefaultZone),
    # Waypoint(Affine(0.67088, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.18162, DefaultZone),
    # Waypoint(Affine(0.67349, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.18318, DefaultZone),
    # Waypoint(Affine(0.6761, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.18478, DefaultZone),
    # Waypoint(Affine(0.67871, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.18643, DefaultZone),
    # Waypoint(Affine(0.68133, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.18813, DefaultZone),
    # Waypoint(Affine(0.68394, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.18989, DefaultZone),
    # Waypoint(Affine(0.68655, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.19171, DefaultZone),
    # Waypoint(Affine(0.68916, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.19359, DefaultZone),
    # Waypoint(Affine(0.69178, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.19554, DefaultZone),
    # Waypoint(Affine(0.69439, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.19756, DefaultZone),
    # Waypoint(Affine(0.697, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.19965, DefaultZone),
    # Waypoint(Affine(0.69961, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.20182, DefaultZone),
    # Waypoint(Affine(0.70222, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.20407, DefaultZone),
    # Waypoint(Affine(0.70484, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.20641, DefaultZone),
    # Waypoint(Affine(0.70745, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.20884, DefaultZone),
    # Waypoint(Affine(0.71006, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.21137, DefaultZone),
    # Waypoint(Affine(0.71267, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.21401, DefaultZone),
    # Waypoint(Affine(0.71529, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.21676, DefaultZone),
    # Waypoint(Affine(0.7179, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.21964, DefaultZone),
    # Waypoint(Affine(0.72051, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.22264, DefaultZone),
    # Waypoint(Affine(0.72312, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.22579, DefaultZone),
    # Waypoint(Affine(0.72573, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.22908, DefaultZone),
    # Waypoint(Affine(0.72835, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.23255, DefaultZone),
    # Waypoint(Affine(0.73096, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.23619, DefaultZone),
    # Waypoint(Affine(0.73357, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.24002, DefaultZone),
    # Waypoint(Affine(0.73618, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.24406, DefaultZone),
    # Waypoint(Affine(0.73879, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.24834, DefaultZone),
    # Waypoint(Affine(0.74141, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.25286, DefaultZone),
    # Waypoint(Affine(0.74402, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.25766, DefaultZone),
    # Waypoint(Affine(0.74663, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.26278, DefaultZone),
    # Waypoint(Affine(0.74924, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.26823, DefaultZone),
    # Waypoint(Affine(0.75186, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.27405, DefaultZone),
    # Waypoint(Affine(0.75447, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.28031, DefaultZone),
    # Waypoint(Affine(0.75708, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.28704, DefaultZone),
    # Waypoint(Affine(0.75969, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.29431, DefaultZone),
    # Waypoint(Affine(0.7623, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.3022, DefaultZone),
    # Waypoint(Affine(0.76492, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.31079, DefaultZone),
    # Waypoint(Affine(0.76753, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.3202, DefaultZone),
    # Waypoint(Affine(0.77014, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.33056, DefaultZone),
    # Waypoint(Affine(0.77275, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.34205, DefaultZone),
    # Waypoint(Affine(0.77537, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.35488, DefaultZone),
    # Waypoint(Affine(0.77798, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.36934, DefaultZone),
    # Waypoint(Affine(0.78059, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.38579, DefaultZone),
    # Waypoint(Affine(0.7832, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.40473, DefaultZone),
    # Waypoint(Affine(0.78581, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.42687, DefaultZone),
    # Waypoint(Affine(0.78843, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.4532, DefaultZone),
    # Waypoint(Affine(0.79104, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.48525, DefaultZone),
    # Waypoint(Affine(0.79365, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.52542, DefaultZone),
    # Waypoint(Affine(0.79626, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.57784, DefaultZone),
    # Waypoint(Affine(0.79888, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.65035, DefaultZone),
    # Waypoint(Affine(0.80149, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.76037, DefaultZone),
    # Waypoint(Affine(0.8041, 0, 0.22672, 0, 0.41132, 0.91149, 0), 0.94114, DefaultZone),
    # Waypoint(Affine(0.80671, 0, 0.22672, 0, 0.41132, 0.91149, 0), 1),
  ])
  robot.move(DefaultTool, motion, data)

program()
