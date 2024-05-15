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
        self.new_image = True
        self.cap = cv2.VideoCapture(0)
        time.sleep(5)
        self.run()

    def action_check(self, data):
        self.new_image = True
    
    @staticmethod
    def gs_threshold(x, threshold=0.35):
        x = torch.where(x <= threshold, torch.zeros_like(x), x)
        return x
    
    def run(self):
        transform = v2.Compose([
            v2.Lambda(lambda x: v2.functional.invert(x)),
            v2.Grayscale(),
            #v2.Lambda(lambda x: self.gs_threshold(x)),
            #v2.Grayscale(),
            v2.Resize((28, 28)),
            v2.RandomRotation(degrees=(90, 90)),
            v2.RandomVerticalFlip(p=1.0),
            #v2.Lambda(lambda x: v2.functional.invert(x)),
            v2.ToImage(),
            v2.ToDtype(torch.float32, scale=True),
            v2.Lambda(lambda x: self.gs_threshold(x)),
            v2.Normalize((0.1307,), (0.3081,))
            ])
        
        mod = torch.load("letter_model2.pt")
        mod.eval()

        while True:

            ret, frame = self.cap.read()
            if not ret:
                print("Error capturing frame")
                break

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

            # Display the camera feed
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)
            """
            # Convert the frame to PIL image
            # Convert the image to HSV format
            hsv_image = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
            plt.imshow(hsv_image)
            plt.savefig('image6-threshold.png')
            plt.show()
            """
            # Split the HSV image into its components
            hsv_image = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
            """h, s, v = cv2.split(hsv_image)
            v *= 2 
            for i in range(len(v)):
                for j in range(len(v[0])):
                    v[i][j] = min(255, v[i][j])
            hsv_image = cv2.merge([h, s, v])"""

            rgb_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)
            pil_image = Image.fromarray(rgb_image)
            transformed_image = transform(pil_image)
            plt.imshow(torch.squeeze(transformed_image), cmap='gray')
            plt.savefig('image6-ringlight.png')
            plt.show()

            # Predict the letter
            with torch.no_grad():
                output = mod(transformed_image)
            probs = torch.softmax(output, dim=1)
            predicted_class = torch.argmax(probs, dim=1).item()
            print("This is the class: ", predicted_class)
            letter = self.alphabet[predicted_class]

            print("This is the letter: ", letter)
            time.sleep(1)
            
if __name__ == '__main__':
    demo = GuessCam()
