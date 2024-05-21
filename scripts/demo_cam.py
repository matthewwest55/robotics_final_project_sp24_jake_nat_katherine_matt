#!/usr/bin/env python3

import rospy, cv2, cv_bridge, torch
import numpy as np
from final_project.msg import GuessedLetter
from std_msgs.msg import String
import matplotlib.pyplot as plt
from PIL import Image
from model import CNN
from torchvision.transforms import v2
 

class GuessCam(object):
    def __init__(self):
        self.alphabet = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 
                         'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 
                         'U', 'V', 'W', 'X', 'Y', 'Z']
        self.guesses = []
        # Tracks if we are looking for a guess
        self.new_image = True
        # Part of control to ignore first guess
        self.first = True
        self.cap = cv2.VideoCapture(0)
        self.check_guess = rospy.Subscriber('need_guess', String, 
                                            self.action_check)
        self.guess_pub = rospy.Publisher('user_guess', GuessedLetter, 
                                         queue_size=1)
        rospy.sleep(.1) # Rospy TODO
        self.run()

    def action_check(self, data):
        rospy.sleep(.1) # TODO: Delet4e]
        if (data != None): 
            self.new_image = True
        else:
            self.new_image = False    

    @staticmethod
    def gs_threshold(x, threshold=0.38):
        x = torch.where(x <= threshold, torch.zeros_like(x), x)
        return x
    
    def run(self):

        curr_guess = "Null"
        curr_count = 0

        transform = v2.Compose([
            v2.Lambda(lambda x: v2.functional.invert(x)),
            v2.Grayscale(),
            v2.Resize((28, 28)),
            v2.RandomRotation(degrees=(90, 90)),
            v2.RandomVerticalFlip(p=1.0),
            v2.ToImage(),
            v2.ToDtype(torch.float32, scale=True),
            v2.Lambda(lambda x: self.gs_threshold(x)),
            v2.Normalize((0.1307,), (0.3081,))
            ])
        
        model = torch.load("/home/jake/catkin_ws/src/robotics_final_project/scripts/letter_model2.pt")
        model.eval()

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                print("Error capturing frame")
                break

            if self.new_image:

                # Assuming frame is the image captured from your camera
                height, width, _ = frame.shape

                # Calculate the size of the square region to crop
                size = min(height, width)

                # Calculate the coordinates for cropping
                top = int(height * 0.4)
                bottom = int(height * .6
                            )
                left = int((width / 2)-(height*.1))
                right = int((width/ 2) + (height*.1))
                cropped = frame[top:bottom, left:right]


                rgb_image = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
                pil_image = Image.fromarray(rgb_image)
                transformed_image = transform(pil_image)
                # Visualize processed image (for debugging)
                
                """plt.imshow(torch.squeeze(transformed_image), cmap='gray')
                plt.savefig('image6-ringlight.png')
                plt.show()"""

                # Predict the letter
                with torch.no_grad():
                    output = model(transformed_image)
                probs = torch.softmax(output, dim=1)
                predicted_class = torch.argmax(probs, dim=1).item()

                letter = self.alphabet[predicted_class]

                if letter == curr_guess:
                    curr_count += 1
                else:
                    if not(curr_guess == "Null"):
                        #print("Letter:",letter)
                        self.first = False
                    curr_count = 1
                    curr_guess = letter
                #print(f"[{curr_count}/7] Prediction: {letter}.")

                if (curr_count == 7) and not (curr_guess in self.guesses):
                    if self.first:
                        curr_count = 1
                    else:
                        # send off msg
                        self.guesses.append(letter)
                        self.new_image = False
                        print(f"[!!!] Letter Guessed: {letter}.")
                        self.guess_pub.publish(curr_count)

            rospy.sleep(.5)
                
            

if __name__ == '__main__':
    demo = GuessCam()