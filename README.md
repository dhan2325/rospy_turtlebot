# rospy_turtlebot

I had the opportunity to work (with a partner) with a [Turtlebot Waffle-Pi](https://www.roscomponents.com/en/mobile-robots/215-turtlebot-3-waffle.html#/courses-no/turtlebot3_waffle_pi_model-waffle_pi) in a school lab and program it to perform various tasks. The labs were focused on pose control, path following, and Kalman Filtering. Combining those three concepts, the final culminating task was to program the Turtlebot to perform mail delivery.

The scripts are orgnized into the separate labs that they were a part of. Please read the documentation for each lab to udnerstand each task and how it was implemented.

The Turtlebots run ROS 1, and the rospy module in Python was used to direct the robots. All scripts in the repository are in Python 3.


# Lab 1: basic commands
Basic motion commands were sent to the turtlebot as twist messages (defining a forward linear velocity and rotational velocity about the vertical axis). The lab focused on running multiple nodes concurrently to use an odometer node in parallel with a motor node and thereby keep track of the position of the turtlebot

# Lab 2: pose control
The turtlebot is able to rotate on the spot (turning radius of 0) and move in a straight line or move on a curved path by varying the individual inputs sent to the two wheels on the turtlebot. The lab focuses on controlling pose in four different scanrios:
1. making full use of the turtlebot's ability to turn on a point, move from one pose to another using a straight line path and point rotations.
2. still with no restrictions imposed on the robots motion, have the turtlebot move between waypoints that implicity define a path for the turtlebot.
3. impose a restriction on the robot's minimum turning radius, then determine a path that the robot can follow to move between two poses
4. contemplate how to have the turtlebot follow a path defined by a sine function, and more generally by an arbitrary differentiable function

# Lab 3: path following and PID control
This lab makes use of the Turtlebot's camera to have the turtlebot follow a line of black tape on the floor. A node is run on the turtlebot to convert greyscale image captured from an array into a single row using weighted averages, then return the index of the 'darkest' column. With this camera node, students were tasked with getting a turtlebot to follow a line of black tape on the floor by implementing various controllers. To do so, the darkest index returned by the camera node was compared to the desired position of the robot (ideally, it would stay close to the center of the line) and their difference considered the 'error' in our system.

The lab focused on PID controllers and their variants. The tasks included implementing a:
* 'bang bang' controller which would give one of two fixed inputs to our robot depending on if the robot needed to turn left or turn right.
* P controller, where the input to the turtlebot was only proportional to the error.
* PI controller, which adds to the P control a term that is dependant on the integral of the error over time
* PID controller, which in turn adds a term dependant on the derivative of the error

Students were also given a chance to implement a controller of their own design. Our own controller implemented a couple of changed from the PID control:
* We began by defining the minimum turning radius our path following control could output by deriving a proportion between maximum linear and rotational velocities
* the term proporional to the error was changed to be proportional to the square of the error. This was found to prevent overshooting but also allow for sharp turns to be made with less difficulty
