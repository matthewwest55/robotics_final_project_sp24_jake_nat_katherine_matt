#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math

class MoveArm(object):
    def __init__(self):
        # initialize this node (get rid of this later)
        rospy.init_node('move_arm')

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        rospy.sleep(2)

        self.reset_arm()
        rospy.sleep(5)
        self.open_gripper()

        # Reset arm position
        # self.move_group_arm.go([0,0,0,0], wait=True)
        print("ready")

    def draw_galley(self, starting_index, matrix):
        #starting_index is a tuple bottom left corner of the galley
        #matrix is the matrix of robot arm positions
        # In this function, we are going to have to pay mind to
        # moving multiple joints at once
        # Example: If the arm is high and begins moving down,
        # it needs to extend a bit to keep touching the board
        
        # Maybe we can make the joints a function of one another?

        #this portion draws the galley base
        x, y = starting_index
        for cell in range(0, 10):
            pose_position = matrix[x + cell][y]
            self.move_group_arm.go(pose_position)

        #reset the default position here
        ################################

        #this portion draws the galley beam
        for cell in range(0, 23):
            pose_position = matrix[x + 5][y + cell]
            self.move_group_arm.go(pose_position)

        #this portion draws the galley top beam
        for cell in range(0, 11):
            pose_position = matrix[x + cell + 5][y + 22]
            self.move_group_arm.go(pose_position)

        #this portion draws the hanging portion of the galley
        for cell in range(0, 6):
            pose_position = matrix[x + 15][y - cell + 22]
            self.move_group_arm.go(pose_position)

    def draw_head(self, matrix, starting_position):
        pass

    def draw_body(self, matrix, starting_position):
        pass

    def draw_left_arm(self, matrix, starting_position):
        pass

    def draw_right_arm(self, matrix, starting_position):
        pass
    
    def draw_left_leg(self, matrix, starting_position):
        pass

    def draw_right_leg(self, matrix, starting_position):
        pass

    def reset_arm(self):
        # left/right, whole arm up/down, forearm up/down, gripper angle
        self.move_group_arm.go([0,math.radians(-20),math.radians(-10),0], wait=True)
        self.move_group_arm.stop()

    def extend_arm(self):
        # self.move_group_arm.go([0, math.radians(15), math.radians(-15), 0], wait=True)
        # self.move_group_arm.go([0, math.radians(30), math.radians(-30), 0], wait=True)
        self.move_group_arm.go([0, math.radians(55), math.radians(-50), 0], wait=True)
        self.move_group_arm.stop()

    def retract_arm(self):
        self.move_group_arm.go([0, math.radians(-55), math.radians(10), 0], wait=True)
        self.move_group_arm.stop()

    def close_gripper(self):
        gripper_joint_close = [-0.005, -0.005]

        self.move_group_gripper.go(gripper_joint_close)
        self.move_group_gripper.stop()

    def open_gripper(self):
        gripper_joint_open = [0.015, 0.015]

        self.move_group_gripper.go(gripper_joint_open)
        self.move_group_gripper.stop()

if __name__ == "__main__":
    arm_commander = MoveArm()
    # arm_commander.extend_arm()
    # arm_commander.retract_arm()
    arm_commander.reset_arm()
    # arm_commander.close_gripper()
    # arm_commander.open_gripper()