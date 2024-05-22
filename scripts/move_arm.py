#!/usr/bin/env python3

import os 
import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import numpy as np
import math

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

        dir_path = os.path.dirname(os.path.realpath(__file__))
        # Skip first row (header) with skiprows
        joint_positions_csv = np.loadtxt(dir_path + "/correct_size_2.csv", delimiter=",", skiprows=1)

        # knowing the matrix is a square, we can take the sqaure root to get its dimensions
        square_dim = int(np.sqrt(len(joint_positions_csv)))

        # The file is ordered from the bottom-right corner to the top-right
        # Then, it moves left one position and goes again
        # To make the matrix start at the top-left, we will reverse the ordering
        self.matrix = np.ndarray((square_dim, square_dim), dtype=np.ndarray)
        for col in range(square_dim - 1, -1, -1):
            for row in range(square_dim - 1, -1, -1):
                # print("Col: " + str(col * square_dim) + " row: " + str(row) + " final: " + str(col*square_dim + row))
                self.matrix[square_dim - 1 - col][square_dim - 1 - row] = joint_positions_csv[(col*square_dim) + row][3:7]

        rospy.sleep(2)

        # self.reset_arm()
        # rospy.sleep(5)
        # self.open_gripper()
        self.close_gripper()

        # Reset arm position
        # self.move_group_arm.go([0,0,0,0], wait=True)
        
    def oriented(self):
        input("Orient now. Type any key to continue.")

    def man_draw(self, remaining):
        # Need to update matrix indices to values associated w each limb
        # These values should be static
        #placeholder = 0 # !!! DELETE -> commented out
        #NOTE: these values can be updated depending on where the arm draws best, and each "placeholder"
        #should be translated the same if we do choose to move the gallows or body.
        #overcorrection adjustments might also need to be made
        if remaining == 5:
            self.draw_head((26, 9))
            #(19,9) ->translated to center for Matt
        elif remaining == 4:
            self.draw_body((26, 15))
            #(19,15) ->translated to center for Matt
        elif remaining == 3:
            self.draw_left_arm((26, 19))
            #(19,19) ->translated to center for Matt
        elif remaining == 2:
            self.draw_right_arm((26, 19))
            #(19,19) ->translated to center for Matt
        elif remaining == 1:
            self.draw_left_leg((26, 21))
            #(19,21) ->translated to center for Matt
        else:
            self.draw_right_leg((26, 21))
            #(19,21) ->translated to center for Matt

    def letter_index(self, num):
        # num is a value in [0,4] (for 5-letter secret word)
        # given num, return matrix index for drawing the letter
        
        #the return tuple is the bottom left corner of each letter, and (ideally)
        #each letter function should just need the bottom left corner location
        #and will adjust from
        if num == 0:
            return (34, 4)
        elif num == 1:
            return (34, 11)
        elif num == 2:
            return (34, 18)
        elif num == 3:
            return (34, 25)
        elif num == 4:
            return (34, 32)
        else:
            print("invalid num value passed")
            return 0
    
    
    def letter_draw(self, letter, mat_ind):
        if letter == "A":
            self.draw_A(mat_ind)
        elif letter == "B":
            self.draw_B(mat_ind)
        # elif letter == "C":
        #    self.draw_C(mat_ind)
        elif letter == "D":
            self.draw_D(mat_ind)
        elif letter == "E":
            self.draw_E(mat_ind)
        elif letter == "F":
            self.draw_F(mat_ind)
        # elif letter == "G":
        #     self.draw_G(mat_ind)
        elif letter == "H":
            self.draw_H(mat_ind)
        elif letter == "I":
            self.draw_I(mat_ind)
        # elif letter == "J":
        #    self.draw_J(mat_ind)
        elif letter == "K":
            self.draw_K(mat_ind)
        elif letter == "L":
            self.draw_L(mat_ind)
        elif letter == "M":
            self.draw_M(mat_ind)
        elif letter == "N":
            self.draw_N(mat_ind)
        elif letter == "O":
            self.draw_O(mat_ind)
        elif letter == "P":
            self.draw_P(mat_ind)
        # elif letter == "Q":
        #    self.draw_Q(mat_ind)
        elif letter == "R":
            self.draw_R(mat_ind)
        # elif letter == "S":
        #    self.draw_S(mat_ind)
        elif letter == "T":
            self.draw_T(mat_ind)
        elif letter == "U":
            self.draw_U(mat_ind)
        # elif letter == "V":
        #     self.draw_V(mat_ind)
        # elif letter == "W":
        #     self.draw_W(mat_ind)
        # elif letter == "X":
        #     self.draw_X(mat_ind)
        # elif letter == "Y":
        #     self.draw_Y(mat_ind)
        # elif letter == "Z":
        #     self.draw_Z(mat_ind)



    # ALL Gallows, Letter, and Person Drawing Code is Below
    ################################################################################################
        

        

    def draw_galley(self, starting_index):
        #for entire matrix, assume (0,0) is the top left corner
        #starting_index is a tuple bottom left corner of the galley
        #matrix is the matrix of robot arm positions

        #this portion draws the galley base
        x, y = starting_index

        # Get into the starting position and wait for the go signal
        pose_position = self.matrix[x + 10][y + 27]
        self.move_group_arm.go(pose_position, wait=True)
        self.oriented()

        for cell in range(0, 10):
            pose_position = self.matrix[x + 10 + cell][y + 27]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(0.5)

        #this portion draws the galley beam
        for cell in range(0, 23):
            pose_position = self.matrix[x + 15][y + 27 - cell]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(0.5)

        # #this portion draws the galley top beam
        for cell in range(0, 11):
            pose_position = self.matrix[x + 15 - cell][y + 5]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(0.5)

        # #this portion draws the hanging portion of the galley
        for cell in range(0, 6):
            pose_position = self.matrix[x + 5][y + 5 + cell]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(0.5)

        rospy.sleep(2)
        self.reset_arm()

    #this portion of code is responsible for drawing the body components
    def draw_head(self, starting_index):
        #starting_index should be the top of the head
        #current starting index = (26, 9)
        #(19,9) ->translated to center for Matt
        x, y = starting_index

        # Get setup
        pose_position = self.matrix[x][y]
        self.move_group_arm.go(pose_position, wait=True)
        rospy.sleep(3)

        # REMOVE LATER
        self.oriented()

        coordinates = [(x + dx, y + dy) for dx, dy in [
            (0, 0), (-1, 0), (-2, 1), (-3, 2),
            (-3, 3), (-3, 4), (-2, 5), (-1, 6),
            (0, 6), (1, 6), (2, 5), (3, 4),
            (3, 3), (3, 2), (2, 1), (1, 0),
            (0, 0)]]

        for coord in coordinates:
            self.move_group_arm.go(self.matrix[coord[0]][coord[1]])
            rospy.sleep(1)


    def draw_body(self, starting_index):
        #current starting index = (26,15)
        #(19,15) ->translated to center for Matt
        x, y = starting_index

        # Get setup
        pose_position = self.matrix[x][y + 7]
        self.move_group_arm.go(pose_position, wait=True)
        rospy.sleep(3)

        for cell in range(0, 7):
            pose_position = self.matrix[x][y + 7 + cell]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(1)

    def draw_left_arm(self, starting_index):
        #current starting index = (26, 19)
        #(19,19) ->translated to center for Matt
        x, y = starting_index

        # Get setup
        pose_position = self.matrix[x][y + 9]
        self.move_group_arm.go(pose_position, wait=True)
        rospy.sleep(3)

        for cell in range(0, 4):
            pose_position = self.matrix[x - cell][y + 9 - cell]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(1)

    def draw_right_arm(self, starting_index):
        #current starting index = (26, 19)
        #(19,19) ->translated to center for Matt
        x, y = starting_index

        # Get setup
        pose_position = self.matrix[x][y + 9]
        self.move_group_arm.go(pose_position, wait=True)
        rospy.sleep(3)

        for cell in range(0, 4):
            pose_position = self.matrix[x + cell][y + 9 - cell]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(1)
    
    def draw_left_leg(self, starting_index):
        #current starting index = (26, 21)
        #(19,21) ->translated to center for Matt
        x, y = starting_index

        # Get setup
        pose_position = self.matrix[x + 1][y + 12]
        self.move_group_arm.go(pose_position, wait=True)
        rospy.sleep(3)

        for cell in range(0, 3):
            pose_position = self.matrix[x + 1 - cell][y + 12 + cell]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(1)

    def draw_right_leg(self, starting_index):
        #current starting index = (26, 21)
        #(19,21) ->translated to center for Matt
        x, y = starting_index

        # Get setup
        pose_position = self.matrix[x - 1][y + 12]
        self.move_group_arm.go(pose_position, wait=True)
        rospy.sleep(3)

        for cell in range(0, 3):
            pose_position = self.matrix[x - 1 + cell][y + 12 + cell]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(1)


    ##This portion is responsible for drawing all alphabet letters
    def draw_A(self, starting_index):
        #bottom left corner
        x, y = starting_index
        for cell in range(0, 4):
            pose_position = self.matrix[x][y - cell]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 2):
            pose_position = self.matrix[x + cell][y - 4]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 4):
            pose_position = self.matrix[x + 3][y + cell]
            self.move_group_arm.go(pose_position, wait=True)

        #reset position

        for cell in range(0, 4):
            pose_position = self.matrix[x + cell][y - 2]
            self.move_group_arm.go(pose_position, wait=True)
            

    def draw_B(self, starting_index):
        #bottom left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = self.matrix[x][y - cell]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y - 4]
            self.move_group_arm.go(pose_position, wait=True)

        self.move_group_arm.go(self.matrix[x + 3][y - 3])

        for cell in range(0, 3):
            pose_position = self.matrix[x + 2 - cell][y - 2]
            self.move_group_arm.go(pose_position, wait=True)

        #reset pose

        self.move_group_arm.go(self.matrix[x + 3][y - 1])

        for cell in range(0, 3):
            pose_position = self.matrix[x + 2 - cell][y]
            self.move_group_arm.go(pose_position, wait=True)

        

    def draw_C(self, starting_index):
        print("letter drawing not implemented")
        pass

    def draw_D(self, starting_index):
        #bottom left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = self.matrix[x][y - cell]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y - 4]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 3):
            self.move_group_arm.go(self.matrix[x + 3][y - 3 + cell])
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 3):
            pose_position = self.matrix[x + 3 - cell][y]
            self.move_group_arm.go(pose_position, wait=True)


    def draw_E(self, starting_index):
        #top left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = self.matrix[x][y + cell]
            self.move_group_arm.go(pose_position, wait=True)

        #reset position
        for cell in range(0, 4):
            pose_position = self.matrix[x + cell][y]
            self.move_group_arm.go(pose_position, wait=True)

        #reset position
        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y + 2]
            self.move_group_arm.go(pose_position, wait=True)

        #reset position
        for cell in range(0, 4):
            pose_position = self.matrix[x + cell][y + 4]
            self.move_group_arm.go(pose_position, wait=True)


    def draw_F(self, starting_index):
        #top left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = self.matrix[x][y + cell]
            self.move_group_arm.go(pose_position, wait=True)

        #reset position
        for cell in range(0, 4):
            pose_position = self.matrix[x + cell][y]
            self.move_group_arm.go(pose_position, wait=True)

        #reset position
        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y + 2]
            self.move_group_arm.go(pose_position, wait=True)

    def draw_G(self, starting_index):
        print("letter drawing not implemented")
        pass

    def draw_H(self, starting_index):
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = self.matrix[x][y + cell]
            self.move_group_arm.go(pose_position, wait=True)

        #reset position
        for cell in range(0, 4):
            pose_position = self.matrix[x + cell][y + 3]
            self.move_group_arm.go(pose_position, wait=True)
        
        #reset position
        for cell in range(0, 5):
            pose_position = self.matrix[x + 3][y + cell]
            self.move_group_arm.go(pose_position, wait=True)


    def draw_I(self, starting_index):
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = self.matrix[x][y + cell]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(1)

    def draw_J(self, starting_index):
        print("letter drawing not implemented")
        pass

    def draw_K(self, starting_index):
        #top left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = self.matrix[x][y + cell]
            self.move_group_arm.go(pose_position, wait=True)

        #reset position
        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y + 3 - cell]
            self.move_group_arm.go(pose_position, wait=True)

        #reset position
        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y + 1 + cell]
            self.move_group_arm.go(pose_position, wait=True)

    def draw_L(self, starting_index):
        #bottom left corner
        x, y = starting_index
        y += 5
        for cell in range(0, 5):
            pose_position = self.matrix[x][y + cell]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(1)

        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y + 4]
            self.move_group_arm.go(pose_position, wait=True)
            rospy.sleep(1)

    def draw_M(self, starting_index):
        #bottom left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = self.matrix[x][y - cell]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y - 4 + cell]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 3):
            pose_position = self.matrix[x + 2 + cell][y - 2 - cell]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 5):
            pose_position = self.matrix[x + 4][y - 4 + cell]
            self.move_group_arm.go(pose_position, wait=True)
            

    def draw_N(self, starting_index):
        #bottom left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = self.matrix[x][y - cell]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 4):
            pose_position = self.matrix[x + cell][y - 4 + cell]
            self.move_group_arm.go(pose_position, wait=True)

        #reset position

        for cell in range(0, 5):
            pose_position = self.matrix[x + 3][y - 4 + cell]
            self.move_group_arm.go(pose_position, wait=True)
            

    def draw_O(self, starting_index):
        #bottom left corner
        x, y = starting_index
        #top of O
        x += 2
        y -= 4
        coordinates = [(x + dx, y + dy) for dx, dy in [
            (0, 0), (-1, 0), (-2, 1), (-2, 2),
            (-2, 3), (-1, 4), (0, 4), (1, 3),
            (1, 2), (1, 1), (0, 0)]]

        for coord in coordinates:
            self.move_group_arm.go(self.matrix[coord[0]][coord[1]])

    def draw_P(self, starting_index):
        #bottom left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = self.matrix[x][y - cell]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y - 4]
            self.move_group_arm.go(pose_position, wait=True)

        self.move_group_arm.go(self.matrix[x + 3][y - 3])

        for cell in range(0, 3):
            pose_position = self.matrix[x + 2 - cell][y - 2]
            self.move_group_arm.go(pose_position, wait=True)


    def draw_Q(self, starting_index):
        print("letter drawing not implemented")
        pass

    def draw_R(self, starting_index):
        #bottom left corner
        x, y = starting_index
        for cell in range(0, 5):
            pose_position = self.matrix[x][y - cell]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y - 4]
            self.move_group_arm.go(pose_position, wait=True)

        self.move_group_arm.go(self.matrix[x + 3][y - 3])

        for cell in range(0, 3):
            pose_position = self.matrix[x + 2 - cell][y - 2]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y - 3 + cell]
            self.move_group_arm.go(pose_position, wait=True)

    def draw_S(self, starting_index):
        print("letter drawing not implemented")
        pass

    def draw_T(self, starting_index):
        #bottom left corner
        x, y = starting_index

        #top left of T
        y -= 4

        for cell in range(0, 3):
            pose_position = self.matrix[x + cell][y]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 5):
            pose_position = self.matrix[x + 1][y + cell]
            self.move_group_arm.go(pose_position, wait=True)

    def draw_U(self, starting_index):
        #bottom left corner
        x, y = starting_index

        #top left corner
        y -= 4
        for cell in range(0, 5):
            pose_position = self.matrix[x][y + cell]
            self.move_group_arm.go(pose_position, wait=True)

        for cell in range(0, 4):
            pose_position = self.matrix[x + cell][y + 4]
            self.move_group_arm.go(pose_position, wait=True)

        #reset position
        for cell in range(0, 5):
            pose_position = self.matrix[x + 3][y + cell]
            self.move_group_arm.go(pose_position, wait=True)
    

    def draw_V(self, starting_index):
        print("letter drawing not implemented")
        pass

    def draw_W(self, starting_index):
        print("letter drawing not implemented")
        pass

    def draw_X(self, starting_index):
        print("letter drawing not implemented")
        pass

    def draw_Y(self, starting_index):
        print("letter drawing not implemented")
        pass

    def draw_Z(self, starting_index):
        print("letter drawing not implemented")
        pass


        # # I'm thinking about making variables to this class 
        # # that allow me to track the current arm position
        # # Using that information, we can move the robot arm slowly
        # # and make movement a function of different joint positions
        
        # # self.arm_down()
        # # rospy.sleep(5)
        # # self.arm_back()

        # # Might also need to make the gripper angle a function of where the arm is
        # # Alright, need to figure out the proportions we should be using

        # # Should be able to get to centered from other values?
        # centered = [math.radians(-90), math.radians(18), math.radians(-2), math.radians(-16)]
        # top_left = [math.radians(-72), math.radians(22), math.radians(-22), math.radians(-21)]
        # top_right = [math.radians(-108), math.radians(22), math.radians(-22), math.radians(-21)]
        # bottom_right = [math.radians(-108), math.radians(43), math.radians(-6), math.radians(-11)]
        # bottom_left = [math.radians(-72), math.radians(43), math.radians(-6), math.radians(-11)]

        # self.move_group_arm.go(centered, wait=True)
        # rospy.sleep(5)
        # self.move_group_arm.stop()

        # self.move_group_arm.go(top_left, wait=True)
        # rospy.sleep(5)
        # self.move_group_arm.stop()

        # self.move_group_arm.go(top_right, wait=True)
        # rospy.sleep(5)
        # self.move_group_arm.stop()

        # self.move_group_arm.go(bottom_right, wait=True)
        # rospy.sleep(5)
        # self.move_group_arm.stop()

        # self.move_group_arm.go(bottom_left, wait=True)
        # rospy.sleep(5)
        # self.move_group_arm.stop()

        # self.move_group_arm.go(top_left, wait=True)
        # rospy.sleep(5)
        # self.move_group_arm.stop()

        # # for i in range(-10, 0, 2):
        # #     # self.move_group_arm.go([RIGHT, math.radians(2*i), math.radians((5*i)+50), 0], wait=True)
        # #     # self.move_group_arm.go([RIGHT, math.radians(-20), math.radians((5*i)+50), 0], wait=True)
        # #     self.move_group_arm.go([RIGHT, math.radians(-2*i), math.radians(-10), 0], wait=True)
        # #     rospy.sleep(2)

        # # self.move_group_arm.stop()


    def reset_arm(self):
        # left/right, whole arm up/down, forearm up/down, gripper angle
        self.move_group_arm.go([0, math.radians(-70), math.radians(70), 0], wait=True)
        self.move_group_arm.stop()
        rospy.sleep(5)

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
        gripper_joint_close = [0.011, 0.011]

        self.move_group_gripper.go(gripper_joint_close)
        self.move_group_gripper.stop()

    def open_gripper(self):
        gripper_joint_open = [0.015, 0.015]

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
    # arm_commander.reset_arm()
    # arm_commander.close_gripper()
    # arm_commander.open_gripper()
