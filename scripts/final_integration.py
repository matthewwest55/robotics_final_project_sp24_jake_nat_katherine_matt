#!/usr/bin/env python3

import rospy

from move_arm import MoveArm
from hangman import Hangman
from final_project.msg import GuessedLetter
from std_msgs.msg import String

class FinalIntegration:
    def __init__(self):
        # Start rospy node
        rospy.init_node("play_hangman")

        # Create modules for playing game 
        self.arm_commands_node = MoveArm()

        rospy.Subscriber('user_guess', GuessedLetter,  self.guessed)
        self.guess_pub = rospy.Publisher('need_guess', String, queue_size=1) 
        wordlist = ["ABODE", "BEARD", "BIKES", "STARK", "SHIFT", "FORKS", "DEPTH", "PRISM", "FRAME", "KNEAD", "DRINK", "BIOME", "THANK", "SPEAK", "POKER", "EMBED","MONTH","MOTIF","HARKS","NORTH"]
        self.game = Hangman(wordlist, 6)
        self.arm_commands_node.draw_galley()
        self.guess_pub.publish(String(data = "Gallows Done"))

        # Sleep to let everything setup
        rospy.sleep(2)

        self.game_over = False
        print("Ready for guesses!")


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
                        mat_ind = self.arm_commands_node.letter_index(i)
                        self.arm_commands_node.letter_draw(data, mat_ind)
                    self.guess_pub.publish(String(data = "Correct guess passed"))    
                else:
                    self.arm_commands_node.man_draw(self.game.remaining_guesses)
                    self.guess_pub.publish(String(data = "Incorrect guess passed"))
            else: 
                self.game_over = True
                print("Restart to play a new game.")