# ROS INDEPENDENT DEMO FILE

import cv2
import numpy as np
import time
from PIL import Image
from model import CNN
import torch
import matplotlib.pyplot as plt
import torchvision
from torchvision.transforms import v2
 
class GuessCam(object):
    def __init__(self):
        self.alphabet = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z']
        self.guesses = []
        # Tracks if we are looking for a guess
        self.new_image = True
        # Part of control to ignore first guess
        self.first = True
        self.cap = cv2.VideoCapture(0)
        time.sleep(.1) # Rospy TODO
        self.run()

    def action_check(self, data):
        time.sleep(1) # TODO: Delet4e] 
        self.new_image = True

    # Threshold is camera dependent. .45 is solid on Macs. .38 is solid
    # Asus Zephyrus machines. Modify between these values or slightly 
    # under/above as needed. 
    @staticmethod
    def gs_threshold(x, threshold=0.45):
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
        
        mod = torch.load("letter_model2.pt")
        mod.eval()

        while True: # This should eventually be rospy shutdown check TODO
            ret, frame = self.cap.read()
            if not ret:
                print("Error capturing frame")
                break

            # Display the camera feed
            # cv2.imshow("Camera Feed", frame)
            # cv2.waitKey(500)

            if self.new_image == True:

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
                    output = mod(transformed_image)
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

                if (curr_count == 9) and not (curr_guess in self.guesses):
                    if self.first:
                        curr_count = 1
                    else:
                        # send off msg
                        self.guesses.append(letter)
                        self.new_image = False
                        print(f"[!!!] Letter Guessed: {letter}.")
                        self.action_check(curr_count) # TODO: delete

            time.sleep(.5) # Rospy TODO



            
if __name__ == '__main__':
    demo = GuessCam()
