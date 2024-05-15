#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
from final_project.msg import GuessedLetter

from PIL import Image
from model import CNN
import torch

import torchvision
from torchvision.transforms import v2
 

class GuessCam(object):
    def __init__(self):
        rospy.init_node('guess_letter')
        rospy.Subscriber('finished_action', GuessedLetter, self.action_check)
        self.letter_pub = rospy.Publisher('guessed_letter', GuessedLetter, queue_size=1)
        # Super smart solution to getting letters:
        self.alphabet = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z']
        self.guesses = []
        # Tracks if we are looking for a guess
        self.new_image = True
        # Part of control to ignore first guess
        self.first = True
        self.cap = cv2.VideoCapture(0)
        rospy.sleep(.1) 
        self.run()

    def action_check(self, data):
        self.new_image = True

    @staticmethod
    def gs_threshold(x, threshold=0.5):
        x = torch.where(x <= threshold, torch.zeros_like(x), x)
        return x
    
    def run(self):

        transform = v2.Compose([
            v2.Lambda(lambda x: v2.functional.invert(x)),
            v2.Grayscale(),
            v2.Resize((28, 28)),
            v2.ToImage(),
            v2.ToDtype(torch.float32, scale=True),
            v2.Lambda(lambda x: self.gs_threshold(x)),
            v2.Normalize((0.1307,), (0.3081,))
            ])
        
        model = torch.load("letter_model2.pt")
        model.eval()

        while not rospy.is_shutdown():

            if self.new_image == True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Error capturing frame")
                    break

                # Assuming frame is the image captured from your camera
                height, width, _ = frame.shape


                # Calculate the coordinates for cropping
                top = int(height * 0.4)
                bottom = int(height * .6)
                left = int((width / 2)-(height*.1))
                right = int((width/ 2) + (height*.1))
                cropped = frame[top:bottom, left:right]
                # From opencv BGR to PIL
                rgb_image = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
                pil_image = Image.fromarray(rgb_image)

                # Display the camera feed
                cv2.imshow("Camera Feed", frame)
                cv2.waitKey(.5)

                # Guessed Letter
                transformed_image = transform(pil_image)
                with torch.no_grad():
                    output = model(transformed_image)
                probs = torch.softmax(output, dim=1)
                predicted_class = torch.argmax(probs, dim=1).item()
                letter = self.alphabet[predicted_class]

                # update guess status
                if letter == curr_guess:
                    curr_count += 1
                else:
                    self.first = False
                    curr_count = 1
                    curr_guess = letter
                # print(f"[{curr_count}/4] Prediction: {letter}.")

                # Basically, if letter has been same on board for 2 seconds
                # and is unguessed (and if we're waiting for a guess)
                # lodge the guess
                if (curr_count == 5) and not (curr_guess in self.guesses):
                    if self.first:
                        # this takes care of init board state, with nothing written
                        curr_count = 1
                    else:
                        # Send off guess
                        lettermsg = GuessedLetter()
                        lettermsg.letter = letter
                        self.guess_pub.publish(lettermsg)
                        # Update guess
                        self.guesses.append(letter)
                        self.new_image = False
                        print(f"[!!!] Letter Guessed: {letter}.")

            rospy.sleep(.5)
                
            

if __name__ == '__main__':
    demo = GuessCam()