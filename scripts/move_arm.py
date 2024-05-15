#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math

RIGHT = math.radians(-90)
LEFT = math.radians(90)

class MoveArm(object):
    def __init__(self):
        # initialize this node (get rid of this later)
        # rospy.init_node('move_arm')

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        rospy.sleep(2)

        # self.reset_arm()
        # rospy.sleep(5)
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
        # I'm thinking about making variables to this class 
        # that allow me to track the current arm position
        # Using that information, we can move the robot arm slowly
        # and make movement a function of different joint positions
        
        # self.arm_down()
        # rospy.sleep(5)
        # self.arm_back()

        # Might also need to make the gripper angle a function of where the arm is
        # Alright, need to figure out the proportions we should be using

        # Should be able to get to centered from other values?
        centered = [math.radians(-90), math.radians(18), math.radians(-2), math.radians(-16)]
        top_left = [math.radians(-72), math.radians(22), math.radians(-22), math.radians(-21)]
        top_right = [math.radians(-108), math.radians(22), math.radians(-22), math.radians(-21)]
        bottom_right = [math.radians(-108), math.radians(43), math.radians(-6), math.radians(-11)]
        bottom_left = [math.radians(-72), math.radians(43), math.radians(-6), math.radians(-11)]

        self.move_group_arm.go(centered, wait=True)
        rospy.sleep(5)
        self.move_group_arm.stop()

        self.move_group_arm.go(top_left, wait=True)
        rospy.sleep(5)
        self.move_group_arm.stop()

        self.move_group_arm.go(top_right, wait=True)
        rospy.sleep(5)
        self.move_group_arm.stop()

        self.move_group_arm.go(bottom_right, wait=True)
        rospy.sleep(5)
        self.move_group_arm.stop()

        self.move_group_arm.go(bottom_left, wait=True)
        rospy.sleep(5)
        self.move_group_arm.stop()

        self.move_group_arm.go(top_left, wait=True)
        rospy.sleep(5)
        self.move_group_arm.stop()

        # for i in range(-10, 0, 2):
        #     # self.move_group_arm.go([RIGHT, math.radians(2*i), math.radians((5*i)+50), 0], wait=True)
        #     # self.move_group_arm.go([RIGHT, math.radians(-20), math.radians((5*i)+50), 0], wait=True)
        #     self.move_group_arm.go([RIGHT, math.radians(-2*i), math.radians(-10), 0], wait=True)
        #     rospy.sleep(2)

        # self.move_group_arm.stop()


    def reset_arm(self):
        # left/right, whole arm up/down, forearm up/down, gripper angle
        self.move_group_arm.go([0, math.radians(-20), math.radians(-10), 0], wait=True)
        self.move_group_arm.stop()
        self.whole_arm = math.radians(-20)
        self.forearm = math.radians(-10)

    def arm_up(self):
        self.move_group_arm.go([RIGHT, math.radians(-20), math.radians(-50), 0], wait=True)
        self.move_group_arm.stop()
        self.whole_arm = math.radians(-20)
        self.forearm = math.radians(-50)

    def arm_down(self):
        # self.move_group_arm.go([RIGHT, math.radians(55), math.radians(-50), 0], wait=True)
        self.move_group_arm.go([RIGHT, math.radians(-20), math.radians(30), 0], wait=True)
        self.move_group_arm.stop()

    def arm_back(self):
        # self.move_group_arm.go([RIGHT, math.radians(55), math.radians(-50), 0], wait=True)
        self.move_group_arm.go([RIGHT, math.radians(-10), math.radians(-10), 0], wait=True)
        self.move_group_arm.stop()

    def close_gripper(self):
        gripper_joint_close = [-0.005, -0.005]

        self.move_group_gripper.go(gripper_joint_close)
        self.move_group_gripper.stop()

    def open_gripper(self):
        gripper_joint_open = [0.011, 0.011]

        self.move_group_gripper.go(gripper_joint_open)
        self.move_group_gripper.stop()

    def test_position(self):
        # self.move_group_arm.go([0, 0.326095, -0.942478, -0.227004], wait=True)
        # self.move_group_arm.go([0, 0.111236, -0.085272, -0.043222], wait=True)
        # self.move_group_arm.go([-0.606988, 1.175835, -0.563517, -0.352847], wait=True)
        # self.move_group_arm.go([0, 0.262321, 0.100058, 0.045154], wait=True)

        # self.move_group_arm.go([2.64377, -1.685372, -0.942478, -0.227004], wait=True)
        # self.move_group_arm.go([-0.547562, 1.366043, -0.942478, -0.227004], wait=True)
        # self.move_group_arm.go([-0.547562, 0.858441, -0.942478, -0.227004], wait=True)
        # self.move_group_arm.go([-0.497822, 0.511812, -0.942478, -0.227004], wait=True)

        # self.move_group_arm.go([-0.534306, 0.652108, -0.942478, -0.227004], wait=True)
        # self.move_group_arm.go([-0.404722, 1.318972, -0.942478, -0.227004], wait=True)
        # self.move_group_arm.go([-0.454744, 0.713187, -0.942478, -0.227004], wait=True)
        # self.move_group_arm.go([-0.404722, 0.602362, -0.942478, -0.227004], wait=True)
        # self.move_group_arm.go([-0.174976, 0.49199, -0.942478, -0.227004], wait=True)

        self.move_group_arm.go([0, 0.111236, -0.085272, -0.043222], wait=True)
        rospy.sleep(5)
        self.move_group_arm.stop()

if __name__ == "__main__":
    arm_commander = MoveArm()
    # arm_commander.extend_arm()
    # arm_commander.retract_arm()
    arm_commander.reset_arm()
    # arm_commander.close_gripper()
    # arm_commander.open_gripper()