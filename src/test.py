import panda_py


robot = panda_py.Panda("172.22.2.3")


while True:

    curr_orientation = robot.get_orientation(scalar_first = True)
    curr_pos = robot.get_position()
    curr_pose = robot.get_pose()
    curr_jts = robot.get_state()

    print('Or: ', curr_orientation)
    print('Pos: ', curr_pos)
    print('Pose: ', curr_pose)
    print('Jts: ', curr_jts.q)