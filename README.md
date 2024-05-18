# robotics_final_project_sp24_jake_nat_katherine_matt

Group Names: Matthew, Jake, Nat, and Katherine

`Final Project: Robot Hangman`
------------------

**Project Description**
------------------------
Goal:

In this project, we aim to create a Turtlebot that is able to use computer vision and inverse mechanics to play a game of hang-man with a user. The Turtlebot will use computer vision to read a users letter guess after it has been written on a white board, and then employ inverse kinematics to either add to the stick-figure or notate a correct guess.

Why this is interesting:
Incorporating user-robot interaction is something we are interested in exploring, because a dynamic program is being created. The robots actions are dependent on the users guesses, and this project also allowed us to question what comprehensive user-eperience looks like. 

What were we able to accomplish:

Throughout this project, we were able to incorporate a computer vision model capable of recognizing handwriting on a white board with a high level of accuracy. We were also able to develop an inverse kinematics system that allowed for an OpenManipulator arm to draw on a white board without having a visual aid to guide it. We experimented with which combinations of letters were easiest and most accurate to both recognize and draw, and then utilized this downsized alphabet to construct a dictionary of words that can be guessed by a user.


Main components and how they fit together:


**System Architecture**
------------------------
Robot Algorithms & Major Components:
(function descriptions per component)

**ROS Node Diagram**
------------------
(insert image here)

**Execution**
---------------------
(Step-by-step instructions of how to run code)


Marker Attachment Instructions:
------------------------------
Within the "Models" folder you will find a 

Challenges
--------------------------------
Inverse Kinematics:

Deciding which system to use when developing an inverse kinematics system proved incredibly challenging, and it took a few attempts before we developed a system that worked well. Notably, we had to create an IK system that worked without the use of a camera, and in order to best do this, we decided to standardize the distance the TB is stationed away from the wall. We tried three different approaches before deciding on our system:

      1. Use Gazebo + physical estimations and then mathematically determine where the arm should be positioned across an entire grid. This led to issues with accuracy and the math was hard to extrapolate since it was not a linear transformation
      2. Isolating a plane within Gazebo and then calculating the joint positions of the TB model, which led to issues with consistency between points. Just because a configuration was calculated didnt mean it was the optimal one.
      3. We found a MATLAB extension that would calculate the ideal position for a given robotics arm and position in 3D coordinates. We then wrote a script that would calculate that positions across a grid, and adjusted as necessary to get a functioning, accurate matrix of positions

Computer Vision:

Future Work
--------------------------------
In the future, we would hope to make this system more dynamic, so it could set its distance from a parallel plane (or white board) and run the calculations internally to decide what the OpenManipulator arm needs to draw where. We would also like to incorporate more overall fuctionality, such as indicating on the board which letters have already been guessed, or being able to adjust to words of varying lengths.

Takeaways
--------------------------------
