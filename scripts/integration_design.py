# this is a document that outlines how we use the letter_guess callback function
# in the move_arm.py file to run the game logic for hangman

class MoveArm(object):
    def __init__(self):

        # ...

        print("ready")

        # New stuff

        rospy.Subscriber('user_guess', GuessedLetter,  self.guessed)
        self.guess_pub = rospy.Publisher('need_guess', String, queue_size=1) 
        self.game = Hangman()

        self.draw_galley() 
        self.guess_pub.publish(String(data = "gallows Done"))
        self.game_over = False
        

    
    def guessed(self, data):
        if not self.game_over:
            if not self.game.is_game_over():
                print("Word:", self.game.display_word()) # to be deleted if the word drawing works
                print("Attempts remaining:", self.game.remaining_guesses) # To be deleted if hangman works
                print("New guess detected: ", data)
                val = self.game.guess_letter(data) 
                # Handles drawing behavior for correct and incorrect guesses (val == T/F respectively)
                # For correct, could have multiple indices for given letter (hence loop) 
                if val:
                    letter_indices = self.game.index_check(data)
                    for i in letter_indices:
                        mat_ind = self.lookup(i)
                        self.letter_draw(data, mat_ind)
                    self.guess_pub.publish(String(data = "Correct guess passed"))    
                else:
                    self.man_draw(self.game.remaining_guesses)
                    self.guess_pub.publish(String(data = "Incorrect guess passed"))
            else: 
                self.game_over = True
                print("Restart to play a new game.")
      
    


    def man_draw(self, remaining)
        # Need to update matrix indices to values associated w each limb
        # These values should be static
        #placeholder = 0 # !!! DELETE -> commented out
        #NOTE: these values can be updated depending on where the arm draws best, and each "placeholder"
        #should be translated the same if we do choose to move the gallows or body.
        #overcorrection adjustments might also need to be made
        if remaining == 5:
            self.draw_head(self.matrix, (26, 9))
        elif remaining == 4:
            self.draw_body(self.matrix, (26, 15))
        elif remaining == 3:
            self.draw_left_arm(self.matrix, (26, 19))
        elif remaining == 2:
            self.draw_right_arm(self.matrix, (26, 19))
        elif remaining == 1:
            self.draw_left_leg(self.matrix, (26, 21))
        else:
            self.draw_right_leg(self.matrix, (26, 21))

    def mat_ind(self, num):
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
            self.draw_A(self.matrix, mat_ind)
        elif letter == "B":
            self.draw_B(self.matrix, mat_ind)
        elif letter == "C":
            self.draw_C(self.matrix, mat_ind)
        elif letter == "D":
            self.draw_D(self.matrix, mat_ind)
        elif letter == "E":
            self.draw_E(self.matrix, mat_ind)
        elif letter == "F":
            self.draw_F(self.matrix, mat_ind)
        elif letter == "G":
            self.draw_G(self.matrix, mat_ind)
        elif letter == "H":
            self.draw_H(self.matrix, mat_ind)
        elif letter == "I":
            self.draw_I(self.matrix, mat_ind)
        elif letter == "J":
            self.draw_J(self.matrix, mat_ind)
        elif letter == "K":
            self.draw_K(self.matrix, mat_ind)
        elif letter == "L":
            self.draw_L(self.matrix, mat_ind)
        elif letter == "M":
            self.draw_M(self.matrix, mat_ind)
        elif letter == "N":
            self.draw_N(self.matrix, mat_ind)
        elif letter == "O":
            self.draw_O(self.matrix, mat_ind)
        elif letter == "P":
            self.draw_P(self.matrix, mat_ind)
        elif letter == "Q":
            self.draw_Q(self.matrix, mat_ind)
        elif letter == "R":
            self.draw_R(self.matrix, mat_ind)
        elif letter == "S":
            self.draw_S(self.matrix, mat_ind)
        elif letter == "T":
            self.draw_T(self.matrix, mat_ind)
        elif letter == "U":
            self.draw_U(self.matrix, mat_ind)
        elif letter == "V":
            self.draw_V(self.matrix, mat_ind)
        elif letter == "W":
            self.draw_W(self.matrix, mat_ind)
        elif letter == "X":
            self.draw_X(self.matrix, mat_ind)
        elif letter == "Y":
            self.draw_Y(self.matrix, mat_ind)
        elif letter == "Z":
            self.draw_Z(self.matrix, mat_ind)
