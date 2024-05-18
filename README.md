# robotics_final_project_sp24_jake_nat_katherine_matt

Group Names: Matthew, Jake, Nat, and Katherine

`Final Project: Robot Hangman`
------------------

**Project Description**
------------------------
*Goal:*
In this project, we aim to create a Turtlebot that is able to use computer vision and inverse mechanics to play a game of hang-man with a user. The Turtlebot will use computer vision to read a users letter guess after it has been written on a white board, and then employ inverse kinematics to either add to the stick-figure or notate a correct guess.

Why this is interesting:
Incorporating user-robot interaction is something we are interested in exploring, because a dynamic program is being created. The robots actions are dependent on the users guesses, and this project also allowed us to question what comprehensive user-eperience looks like. 

What were we able to accomplish:

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

Challenges
--------------------------------
Inverse Kinematics:

Deciding which system to use when developing an inverse kinematics system proved incredibly challenging, and it took a few attempts before we developed a system that worked well. Notably, we had to create an IK system that worked without the use of a camera, and in order to best do this, we decided to standardize the distance the TB is stationed away from the wall. We tried three different approaches before deciding on our system:

      1. Use Gazebo + physical estimations and then mathematically determine where the arm should be positioned across an entire grid. This led to issues with accuracy and the math was hard to extrapolate since it was not a linear transformation
      2. Isolating a plane within Gazebo and then calculating the joint positions of the TB model, which led to issues with consistency between points. Just because a configuration was calculated didnt mean it was the optimal one.
      3. We found a MATLAB extension that would calculate the ideal position for a given robotics arm and position in 3D coordinates. We then wrote a script that would calculate that positions across a grid, and adjusted as necessary to get a functioning, accurate matrix of positions

Future Work
--------------------------------

Takeaways
--------------------------------
