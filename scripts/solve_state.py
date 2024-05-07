#!/usr/bin/env python3

from move_arm import MoveArm
import rospy

if __name__ == '__main__':
    # Start rospy node
    rospy.init_node("solve_state")

    # Init our robot controller objects
    arm_commands_node = MoveArm()

    # arm_commands_node.pick_up_object()
    # arm_commands_node.drop_object()
    # arm_commands_node.open_gripper()

    rospy.spin()