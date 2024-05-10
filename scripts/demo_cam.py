#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
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
        self.alphabet = ['Null','A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z']
        self.guesses = ['Null']
        self.new_image = True
        self.cap = cv2.VideoCapture(0)
        rospy.sleep(5)
        self.run()

    def action_check(self, data):
        self.new_image = True

    
    def run(self):

        transform = v2.Compose([
            v2.Grayscale(),
            v2.Resize((28, 28)),
            v2.Lambda(lambda x: v2.functional.invert(x)),
            v2.ToImage(),
            v2.ToDtype(torch.float32, scale=True),
            v2.Normalize((0.1307,), (0.3081,))
            ])
        
        model = torch.load("letter_model.pt")
        model.eval()

        while not rospy.is_shutdown():

            ret, frame = self.cap.read()
            if not ret:
                print("Error capturing frame")
                break
            # From opencv BGR to PIL
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(rgb_image)
            transformed_image = transform(pil_image)
            with torch.no_grad():
                output = model(transformed_image)
            letter = self.alphabet[output.item()]

            # Display the camera feed
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)

            lettermsg = GuessedLetter()
            lettermsg.letter = letter
            self.guess_pub.publish(lettermsg)

            """
            if not (letter in self.guesses) and self.new_image:
                self.new_image = False 
                self.guesses.append(letter)
                lettermsg = GuessedLetter()
                lettermsg.letter = letter
                self.guess_pub.publish(lettermsg)
            """
            rospy.sleep(1)
            
            

if __name__ == '__main__':
    demo = GuessCam()