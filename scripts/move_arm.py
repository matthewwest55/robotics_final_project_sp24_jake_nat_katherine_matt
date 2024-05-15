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
            pose_position = matrix[x + 10 + cell][y + 27]
            self.move_group_arm.go(pose_position)

        self.reset_arm()

        #this portion draws the galley beam
        for cell in range(0, 23):
            pose_position = matrix[x + 15][y + 27 - cell]
            self.move_group_arm.go(pose_position)

        #this portion draws the galley top beam
        for cell in range(0, 11):
            pose_position = matrix[x + cell + 5][y + 5]
            self.move_group_arm.go(pose_position)

        #this portion draws the hanging portion of the galley
        for cell in range(0, 6):
            pose_position = matrix[x + 15][y - cell + 22]
            self.move_group_arm.go(pose_position)

    #this portion of code is responsible for drawing the body components
    def draw_head(self, matrix, starting_index):
        x, y = starting_index
        for cell in range(0, 16):
            pass

    def draw_body(self, matrix, starting_index):
        x, y = starting_index
        for cell in range(0, 7):
            pose_position = matrix[x + 27][y + 16 + cell]
            self.move_group_arm.go(pose_position)

    def draw_left_arm(self, matrix, starting_index):
        x, y = starting_index
        for cell in range(0, 3):
            pose_position = matrix[x + 27 - cell][y + 19 - cell]
            self.move_group_arm.go(pose_position)

    def draw_right_arm(self, matrix, starting_index):
        x, y = starting_index
        for cell in range(0, 3):
            pose_position = matrix[x + 27 + cell][y + 19 - cell]
            self.move_group_arm.go(pose_position)
    
    def draw_left_leg(self, matrix, starting_index):
        x, y = starting_index
        for cell in range(0, 3):
            pose_position = matrix[x + 27 - cell][y + 23 + cell]
            self.move_group_arm.go(pose_position)

    def draw_right_leg(self, matrix, starting_index):
        x, y = starting_index
        for cell in range(0, 3):
            pose_position = matrix[x + 27 + cell][y + 23 + cell]
            self.move_group_arm.go(pose_position)

    ##This portion is responsible for drawing all alphabet letters
    #Letters Completed:

    def draw_A(self, matrix, starting_index):

    def draw_B(self, matrix, starting_index):

    def draw_C(self, matrix, starting_index):

    def draw_D(self, matrix, starting_index):

    def draw_E(self, matrix, starting_index):
        #top left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = matrix[x][y + cell]
            self.move_group_arm.go(pose_position)

        #reset position
        for cell in range(0, 4):
            pose_position = matrix[x + cell][y]
            self.move_group_arm.go(pose_position)

        #reset position
        for cell in range(0, 3):
            pose_position = matrix[x + cell][y + 2]
            self.move_group_arm.go(pose_position)

        #reset position
        for cell in range(0, 4):
            pose_position = matrix[x + cell][y + 4]
            self.move_group_arm.go(pose_position)


    def draw_F(self, matrix, starting_index):
        #top left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = matrix[x][y + cell]
            self.move_group_arm.go(pose_position)

        #reset position
        for cell in range(0, 4):
            pose_position = matrix[x + cell][y]
            self.move_group_arm.go(pose_position)

        #reset position
        for cell in range(0, 3):
            pose_position = matrix[x + cell][y + 2]
            self.move_group_arm.go(pose_position)

    def draw_G(self, matrix, starting_index):

    def draw_H(self, matrix, starting_index):
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = matrix[x][y + cell]
            self.move_group_arm.go(pose_position)

        #reset position
        for cell in range(0, 4):
            pose_position = matrix[x + cell][y + 3]
            self.move_group_arm.go(pose_position)
        
        #reset position
        for cell in range(0, 5):
            pose_position = matrix[x + 3][y + cell]
            self.move_group_arm.go(pose_position)


    def draw_I(self, matrix, starting_index):
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = matrix[x][y + cell]
            self.move_group_arm.go(pose_position)

    def draw_J(self, matrix, starting_index):

    def draw_K(self, matrix, starting_index):
        #top left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = matrix[x][y + cell]
            self.move_group_arm.go(pose_position)

        #reset position
        for cell in range(0, 3):
            pose_position = matrix[x + cell][y + 3 - cell]
            self.move_group_arm.go(pose_position)

        #reset position
        for cell in range(0, 3):
            pose_position = matrix[x + cell][y + 1 + cell]
            self.move_group_arm.go(pose_position)

    def draw_L(self, matrix, starting_index):
        #top left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = matrix[x][y + cell]
            self.move_group_arm.go(pose_position)

        for cell in range(0, 3):
            pose_position = matrix[x + cell][y + 4]
            self.move_group_arm.go(pose_position)

    def draw_M(self, matrix, starting_index):
        #bottom left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = matrix[x][y - cell]
            self.move_group_arm.go(pose_position)

        for cell in range(0, 3):
            pose_position = matrix[x + cell][y - 4 + cell]
            self.move_group_arm.go(pose_position)

        for cell in range(0, 3):
            pose_position = matrix[x + 2 + cell][y - 2 - cell]
            self.move_group_arm.go(pose_position)

        for cell in range(0, 5):
            pose_position = matrix[x + 4][y - 4 + cell]
            self.move_group_arm.go(pose_position)
            

    def draw_N(self, matrix, starting_index):
        #bottom left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = matrix[x][y - cell]
            self.move_group_arm.go(pose_position)

        for cell in range(0, 4):
            pose_position = matrix[x + cell][y - 4 + cell]
            self.move_group_arm.go(pose_position)

        #reset position

        for cell in range(0, 5):
            pose_position = matrix[x + 3][y - 4 + cell]
            self.move_group_arm.go(pose_position)
            

    def draw_O(self, matrix, starting_index):

    def draw_P(self, matrix, starting_index):

    def draw_Q(self, matrix, starting_index):

    def draw_R(self, matrix, starting_index):

    def draw_S(self, matrix, starting_index):

    def draw_T(self, matrix, starting_index):
        #top left corner
        x, y = starting_index
        for cell in range(0, 3):
            pose_position = matrix[x + cell][y]
            self.move_group_arm.go(pose_position)

        for cell in range(0, 5):
            pose_position = matrix[x + 1][y + cell]
            self.move_group_arm.go(pose_position)

    def draw_U(self, matrix, starting_index):
        #top left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = matrix[x][y + cell]
            self.move_group_arm.go(pose_position)

        for cell in range(0, 4):
            pose_position = matrix[x + cell][y + 4]
            self.move_group_arm.go(pose_position)

        #reset position
        for cell in range(0, 5):
            pose_position = matrix[x + 3][y + cell]
            self.move_group_arm.go(pose_position)
    

    def draw_V(self, matrix, starting_index):

    def draw_W(self, matrix, starting_index):

    def draw_X(self, matrix, starting_index):

    def draw_Y(self, matrix, starting_index):

    def draw_Z(self, matrix, starting_index):


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