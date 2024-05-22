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
Robot Algorithms & Major Components:
(function descriptions per component) TODO

**ROS Node Diagram**
------------------
(insert image here) TODO

**Execution**
---------------------

How to train CV model:
You do not need to train your own letter recognition CV model to run this code, as we have already included one in the repo. However, if you want to train and save your own Pytorch model, you can just execute the `model.py` file in the scripts folder using Python3. Some key dependencies you may need to install (using pip) are the `Deeplake` and `Pytorch` libraries.

How to run Rospy code:
* On the machine which is running Roscore, set the webcam up, across from the board that you will write your letters on. It works best to tune your "letter writing area" simply by bringing up the webcam and centering an example letter in your feed. Your webcam may be different if it has a different resolution, but on a 2023 Macbook you want to center a 3.5 inch square box 22 inches away from the cam. Ensure there is light source directly overhead the whiteboard.
* Run the action.launch file in the launch folder, using the command `roslaunch *package_name* action.launch`. The game outputs appear in the terminal of the machine you're running the code on.

Marker Attachment Instructions:
------------------------------
Within the "3D_models" folder you will find a file called Robotics_Expo_Holder.gx, this is a print file for the components needed to print a expo-marker holder. The intention behind this print is to give more leeway with an OpenManipulator arm, as our team discovered that drawing on a planar space is incredibly challenging to fine-tune, and as such we created this to give us more room when drawing.


Instructions:

1. To print this model, you must have certification from the MADD Center. You can find their 3D printing exam on their Canvas page.
2. This print is intended for the FlashForge Adventurer 4's, and the print time is about 10 hours (without complications). Fair warning, this did take 4 days to get a successful print, you must supervise your print at all times, and you are not allowed to print overnight. This is essentially a full-day commitment, so make sure you have the time set aside or multiple people take shifts watching the print.
3. Once the model has been printed, make sure to carefully peel it off of the baseplate and remove the supports.
4. Use superglue to construct the model as shown in the photograph, and ensure that everything is applied evenly so it can be gripped well.
5. Experiment with looping various strengths and lengths of rubberbands to get the desired amount of marker wiggle room.

Challenges
--------------------------------
Inverse Kinematics:

Deciding which system to use when developing an inverse kinematics system proved incredibly challenging, and it took a few attempts before we developed a system that worked well. Notably, we had to create an IK system that worked without the use of a camera, and in order to best do this, we decided to standardize the distance the TB is stationed away from the wall. We tried three different approaches before deciding on our system:

      1. Use Gazebo + physical estimations and then mathematically determine where the arm should be positioned across an entire grid. This led to issues with accuracy and the math was hard to extrapolate since it was not a linear transformation
      2. Isolating a plane within Gazebo and then calculating the joint positions of the TB model, which led to issues with consistency between points. Just because a configuration was calculated didnt mean it was the optimal one.
      3. We found a MATLAB extension that would calculate the ideal position for a given robotics arm and position in 3D coordinates. We then wrote a script that would calculate that positions across a grid, and adjusted as necessary to get a functioning, accurate matrix of positions. Even with this we ran into issues regarding placement, size of non-linear offset as the software adjusted for the corners of the plane, and issues drawing point-to-point.

Computer Vision:

The model we trained for letter recognition requires pretty good input conditions to read letters well off of a whiteboard. Knowing them now, these conditions are not prohibitive–we found multiple indoor locations that had enough ambient light for solid processing, and the camera-whiteboard setup described above produces consistent results–but it was quite a challenge to figure out. We also had some issues tuning the complexity of our model, and with overtraining. When the task is as simple (relative to other CV tasks) as classifying nicely-processed data, the overeager addition of many linear layers to our neural net created some very strange classification results: there aren't that many important "things" to pay attention to, and a simpler CNN structure reflects as much. Consequently, this model was very liable to overtrain: we found that after about ten epochs–a training set of about 20,000 images–we would see the network collapse and classify everything as a few letters.


Future Work
--------------------------------
In the future, we would hope to make this system more dynamic, so it could set its distance from a parallel plane (or white board) and run the calculations internally to decide what the OpenManipulator arm needs to draw where. We would also like to incorporate more overall fuctionality, such as indicating on the board which letters have already been guessed, or being able to adjust to words of varying lengths. 

Takeaways
--------------------------------
* Just because you see two classification tasks as the same does not mean that your CV algorithm will. Reading letters off of photocopied paper is not the same as reading them from a live webcam off of a whiteboard. The best way to avoid this is to train your CV model on data you collect yourself. But we were able to close the gap by being particular with our input conditions.
* Integrating two essentially separate projects requires more thorough communication than understanding the outputs of those separate parts. This is particularly true for two pieces of code running simultaneously, and the inevitable synchronicity issues that come with it.
* Dealing with planar math embedded within an inverse kinematics problem is hard. Chosing simplifications to model in–say, instead of viewing the plane as a continuous surface and instead as something like a discrete dot-matrix, and assuming roughly continous behavior in the arm between those points–is very helpful for tackling these problems without a deep knowlege of the subject. 
