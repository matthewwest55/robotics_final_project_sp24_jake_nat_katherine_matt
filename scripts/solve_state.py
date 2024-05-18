#!/usr/bin/env python3

from move_arm import MoveArm
import rospy
import numpy as np
import os

if __name__ == '__main__':
    # Start rospy node
    rospy.init_node("solve_state")

    # # Should be able to get to centered from other values?
    # # centered = [-90, 18, -2, -16]
    # top_left = [-72, 22, -22, -21]
    # top_right = [-108, 22, -22, -21]
    # bottom_right = [-108, 43, -6, -11]
    # bottom_left = [-72, 43, -6, -11]
    

    # path_prefix = os.path.dirname(__file__)
    # # Load the Q-matrix
    # arm_position_matrix = np.loadtxt(os.path.join(path_prefix, "..//", "q-matrix.csv"), delimiter=' ')

    # Init our robot controller objects
    arm_commands_node = MoveArm()

    arm_commands_node.draw_L((20,20))

    # arm_commands_node.draw_line()
    # # arm_commands_node.drop_object()
    # arm_commands_node.open_gripper()

    rospy.spin()