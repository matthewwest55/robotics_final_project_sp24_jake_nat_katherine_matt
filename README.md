# robotics_final_project_sp24_jake_nat_katherine_matt

Group Names: Matthew, Jake, Nat, and Katherine

`Final Project: Robot Hangman`
------------------

**Project Description**
------------------------
**Goal:**

In this project, we aim to create a Turtlebot that is able to use computer vision and inverse mechanics to play a game of hang-man with a user. The Turtlebot will use computer vision to read a users letter guess after it has been written on a white board, and then employ inverse kinematics to either add to the stick-figure or notate a correct guess.

**Why this is interesting:**
Incorporating user-robot interaction is something we are interested in exploring, because a dynamic program is being created. The robots actions are dependent on the users guesses, and this project also allowed us to question what comprehensive user-eperience looks like. 

**What were we able to accomplish:**

Throughout this project, we were able to incorporate a computer vision model capable of recognizing handwriting on a white board with a high level of accuracy. We were also able to develop an inverse kinematics system that allowed for an OpenManipulator arm to draw on a white board without having a visual aid to guide it. We experimented with which combinations of letters were easiest and most accurate to both recognize and draw, and then utilized this downsized alphabet to construct a dictionary of words that can be guessed by a user.

**Main components and how they fit together:**

As previously mentioned, the two main components of this project are an inverse kinematics and a computer vision component. The inverse kinematics portion basically involved creating a modular way to write letters and draw a hangman on a whiteboard at a fixed location. The CV portion involved reading letters off of a whiteboard using a Convolutional Neural Net (CNN) trained using PyTorch on the EMNIST letter dataset. On seeing a letter, our code takes it as a guess, and through an instance of a game of hangman (developed as a class in `hangman.py`), makes some decisions about what to draw on the board (a letter, a hangman's leg, etc.). 

**System Architecture**
------------------------
**Robot Algorithms & Major Components:**

This project focused on implementing two major components, Inverse Kinematics and Computer Vision. The portions of inverse kinematics were concentrated in the `load_matrix.py` and `move_arm.py` file, whereas the computer vision component was implemented within the `demo_cam.py` file. 

----

`hangman.py`

This file contains all of our core logic to play a game of hangman.
  1. `def __init__(self, wordlist, guesses_allowed):` initializes all components of the game hangman.
  2. `def choose_word(self, wordlist)` selects a word off of a predetermined word list to use for the game.
  3. `def display_word(self)` prints the word to the terminal for players to see.
  4. `def guess_letter(self, letter)` handles the guessing of the letter, indicating whether the guess was right or wrong, and drawing the body component accordingly.
  5. `def is_game_over(self)` handles the state of the game, determining whether a game is over based on the number of remaining incorrect guesses.
  6. `def index_check(self, letter)` checks where a correct letter is positioned within the word.

------

`load_matrix.py`

This file contains the necessary code to load the calculated OpenManipulator joint matrix into an accessible form for use. It formats the matrix into a 40x40 grid that can be indexed in (x,y) format, representing a point within our drawing plane.

-----

`move_arm.py`

This file contains all necessary code to draw the appropriate components of the game hangman. All functions operate on the provided `matrix` that serves as a plane for the arm to draw on. The individual functions are brokendown below:
  1. `def __init__(self)` initializes all necessary components to move the OpenManipulator arm.
  2. `def oriented(self)` indicates to players that the robot should be positioned parallel to the surface it will be drawing on.
  3. `def guessed(self, data)` handles the terminal printouts regarding the state of the game, as well as handles either indicating a correct guess, or drawing a component of the hangman.
  4. `def man_draw(self, remaining)` depending on the remaining amount of incorrect guesses, this function indicates which bodypart component to draw.
  5. `def letter_index(self, num)`  this function indicates where in the word the guessed letter is located.
  6. `def letter_draw(self, letter, mat_ind)` signals which letter to draw.
  7. `def draw_galley(self, starting_index)` draws the galley.
  8. `def draw_head(self, starting_index)`, `def draw_body(self, starting_index)`, `def draw_left_arm(self, starting_index)`, `def draw_right_arm(self, starting_index)`, `def draw_left_leg(self, starting_index)`, and `def draw_right_leg(self, starting_index)` draw the respective body components.
  9. `def draw_A(self, starting_index)` -> `def draw_Z(self, starting_index)` indicate how to draw the indicated letter by noting which "cells" should be colored accoridng to a 5x4 square. Essentially, creating pixel letters.

------

`demo_cam.py`

This file contains all code necessary to read images off the whiteboard via openCV and determine what letters (or lack of) are in that image via the computer vision dataset with Pytorch. Functions and their purpose are listed below:
1. `def init(self)` This node initializes necessary components such as the ROS node, the alphabet as a list, an empty list that will hold all the previously guessed letters, the cv2 image, and the rospy publish and subscribers.
2. `def action_check(self, data)` This node will verify if we need to grab a new letter. It is a callback function that relies on the `need_guess` subscriber from ROS. If there is a tangible value, that is, any non-None type object, then it will notify the code that we are ready to get a new letter. Else, it simply rospy.sleeps in a loop.
3. `def gs_threshold(x, threshold=0.38)` This node ensures that the lighter shades of pixels discovered within an image taken from the camera are darkened. That way only the written letter, which is grayscaled and inverted alongside the rest of the image, is the only readable object in the image. 
4. `def run(self)` This node runs the code. That is, it will rospy sleep in a loop if the code does not specify that we need a new guess from the callback function or it will search a given captured image frame from openCV that is processed and matched to a letter given by the CV model and will publish it to the `move_arm.py` rospy node for processing either as an incorrect guess (in which a hangman body part is drawn) or if it is correct (in which a letter is drawn).

----

`model.py`
This file is the CV training model that trains a model using the EMNST dataset using the pytorch and deeplake libraries. This file only needs to be run once and is independent of ROS. As such, you should run this file alone and using python3 rather than using rosrun. 
1. `def init(self)` Initializes necessary modules for the training model.
2. `def forward(self, x)` Describes how the optimization of the model happens.

----

`solve_state.py`
This file demonstrates all aspects of inverse kinematics within our project as a demo, should we need to showcase this particular aspect.

-----

**ROS Node Diagram**
------------------

![image](https://github.com/matthewwest55/robotics_final_project_sp24_jake_nat_katherine_matt/assets/116113433/f035bfaa-8f2f-46fc-8924-0d3ec1ab8a1c)


**Execution**
---------------------

**How to train CV model:**
You do not need to train your own letter recognition CV model to run this code, as we have already included one in the repo. However, if you want to train and save your own Pytorch model, you can just execute the `model.py` file in the scripts folder using Python3. Some key dependencies you may need to install (using pip) are the `Deeplake` and `Pytorch` libraries.

**How to run Rospy code:**
* On the machine which is running Roscore, set the webcam up, across from the board that you will write your letters on. It works best to tune your "letter writing area" simply by bringing up the webcam and centering an example letter in your feed. Your webcam may be different if it has a different resolution, but on a 2023 Macbook you want to center a 3.5 inch square box 22 inches away from the cam. Ensure there is light source directly overhead the whiteboard.
* Run the action.launch file in the launch folder, using the command `roslaunch *package_name* action.launch`. The game outputs appear in the terminal of the machine you're running the code on.

Marker Attachment Instructions:
------------------------------
Within the "3D_models" folder you will find a file called Robotics_Expo_Holder.gx, this is a print file for the components needed to print a expo-marker holder. The intention behind this print is to give more leeway with an OpenManipulator arm, as our team discovered that drawing on a planar space is incredibly challenging to fine-tune, and as such we created this to give us more room when drawing.


**Instructions:**

1. To print this model, you must have certification from the MADD Center. You can find their 3D printing exam on their Canvas page.
2. This print is intended for the FlashForge Adventurer 4's, and the print time is about 10 hours (without complications). Fair warning, this did take 4 days to get a successful print, you must supervise your print at all times, and you are not allowed to print overnight. This is essentially a full-day commitment, so make sure you have the time set aside or multiple people take shifts watching the print.
3. Once the model has been printed, make sure to carefully peel it off of the baseplate and remove the supports.
4. Use superglue to construct the model as shown in the photograph, and ensure that everything is applied evenly so it can be gripped well.
5. Experiment with looping various strengths and lengths of rubberbands to get the desired amount of marker wiggle room.

Challenges
--------------------------------
**Inverse Kinematics:**

Deciding which system to use when developing an inverse kinematics system proved incredibly challenging, and it took a few attempts before we developed a system that worked well. Notably, we had to create an IK system that worked without the use of a camera, and in order to best do this, we decided to standardize the distance the TB is stationed away from the wall. We tried three different approaches before deciding on our system:

1. Use Gazebo + physical estimations and then mathematically determine where the arm should be positioned across an entire grid. This led to issues with accuracy and the math was hard to extrapolate since it was not a linear transformation
2. Isolating a plane within Gazebo and then calculating the joint positions of the TB model, which led to issues with consistency between points. Just because a configuration was calculated didnt mean it was the optimal one.
3. We found a MATLAB extension that would calculate the ideal position for a given robotics arm and position in 3D coordinates. We then wrote a script that would calculate that positions across a grid, and adjusted as necessary to get a functioning, accurate matrix of positions. Even with this we ran into issues regarding placement, size of non-linear offset as the software adjusted for the corners of the plane, and issues drawing point-to-point.

**Computer Vision:**

The model we trained for letter recognition requires pretty good input conditions to read letters well off of a whiteboard. Knowing them now, these conditions are not prohibitive–we found multiple indoor locations that had enough ambient light for solid processing, and the camera-whiteboard setup described above produces consistent results–but it was quite a challenge to figure out. We also had some issues tuning the complexity of our model, and with overtraining. When the task is as simple (relative to other CV tasks) as classifying nicely-processed data, the overeager addition of many linear layers to our neural net created some very strange classification results: there aren't that many important "things" to pay attention to, and a simpler CNN structure reflects as much. Consequently, this model was very liable to overtrain: we found that after about ten epochs–a training set of about 20,000 images–we would see the network collapse and classify everything as a few letters.

**Hardware Issues:**

Unfortunately, at 6:00pm on Wed, May 22nd (the evening before the final project), our SSD that was running native linux completely died. This drive was how we were going to run the demo, and the code for CV is configured to the camera's resolution from Jake's laptop and also needs to be native. NoMachine did not function with it from what we've tested with, and it essentially crippled our testing pipeline. We came up with some stopgap solutions, essentially in which we would act as the rostopic publisher, but this forced us to reassess how we wanted to move forward.


Future Work
--------------------------------
In the future, we would hope to make this system more dynamic, so it could set its distance from a parallel plane (or white board) and run the calculations internally to decide what the OpenManipulator arm needs to draw where. We would also like to incorporate more overall fuctionality, such as indicating on the board which letters have already been guessed, or being able to adjust to words of varying lengths. 

Takeaways
--------------------------------
* Just because you see two classification tasks as the same does not mean that your CV algorithm will. Reading letters off of photocopied paper is not the same as reading them from a live webcam off of a whiteboard. The best way to avoid this is to train your CV model on data you collect yourself. But we were able to close the gap by being particular with our input conditions.
* Integrating two essentially separate projects requires more thorough communication than understanding the outputs of those separate parts. This is particularly true for two pieces of code running simultaneously, and the inevitable synchronicity issues that come with it.
* Dealing with planar math embedded within an inverse kinematics problem is hard. Chosing simplifications to model in–say, instead of viewing the plane as a continuous surface and instead as something like a discrete dot-matrix, and assuming roughly continous behavior in the arm between those points–is very helpful for tackling these problems without a deep knowlege of the subject. 
