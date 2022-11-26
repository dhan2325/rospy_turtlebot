# rospy_turtlebot

I had the opportunity to work (with a partner) with a [Turtlebot Waffle-Pi](https://www.roscomponents.com/en/mobile-robots/215-turtlebot-3-waffle.html#/courses-no/turtlebot3_waffle_pi_model-waffle_pi) in a school lab and program it to perform various tasks. The labs involved elementary implementation of algorithms relevant to mobile robots, including pose control, path detection and following, Kalman Filtering, and Bayesian Localization.

The scripts are orgnized into the separate labs that they were a part of. Please read the documentation for each lab to udnerstand each task and how it was implemented.

The Turtlebots run ROS 1, and the rospy module in Python was used to direct the robots. All scripts in the repository are in Python 3.

### Lab 1: Introduction
Focus on setting up nodes and topics to interact with the turtlebot and send simple motion commands


## Lab 2: Pose Control
Applied bicycle model to the turtlebot, used varying linear and angular speeds to achieve desired motion in two dimensions.



## Lab 3: Path Following
Path following was achieved by solving an equivalent regulation problem, using PID control to keep robot on the desired line while moving forward.



## Lab 4: Kalman Filtering
Implemented a Kalman Filter to localize the turtlebot using a lidar to detect landmarks as the measurement model and a linearization of position as the state model.



## Final Project: Mail Delivery
The Final project involved implementing a mail delivery system on the turtlebot using elements from previous labs. The robot is programmed to follow a specified path, stop at specified destinations to drop off mail, then return to the starting point by following the path back. The robot uses the same control module as in Lab 3 to follow the specified path, and uses Bayesian Localization to determine where along the path it is situated.
