# rospy_turtlebot

I had the opportunity to work with a [Turtlebot Waffle-Pi](https://www.roscomponents.com/en/mobile-robots/215-turtlebot-3-waffle.html#/courses-no/turtlebot3_waffle_pi_model-waffle_pi) in a school lab and program it to perform various tasks. The labs were focused on pose control, path following, and Kalman Filtering. Combining those three concepts, the final culminating task was to program the Turtlebot to perform mail delivery.

The scripts are orgnized into the separate labs that they were a part of. Please read the documentation for each lab to udnerstand each task and how it was implemented.

The Turtlebots run ROS 1, and the rospy module in Python was used to direct the robots. All scripts in the repository are in Python 3.


# Lab 1
Familiarization with the Turtlebot. Basic motion commands were sent to the turtlebot as twist messages (defining a forward linear velocity and rotational velocity about the vertical axis). There were three predetermined paths that were followed by the robot. First, the turtlebot went forward one meter then performed a 360 degree rotation. Next, the turtlebot traced out a square, finishing in its initial pose. Finally, a minimum turning radius was imposed on the turtlebot, and then the turtlebot was tasked with moving between two predetermined poses with this new restriction.
