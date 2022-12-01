# Lab 4: Kalman Flitering

This lab makes use of the Turtlebot's Lidar System to measure the relative position of a landmark and uses this data to implement a Kalman Filter that can localize the TurtleBot.

The TurtleBot was made to follow a straight line (using SID control as outlined lab3.md) and our task was to be able to localize the robot using an odometry system model and an angular measurement model. The Robot's position was estimated in each iteration by assuming a constant velocity between iterations and calculating where the robot's next position was expected to be. This prediction was updated by measuring the relative angle between the robot's direction of motion and the landmark, which could be used to determine which position would be expected to yield the observed measurement.

In the steup for the Kalman filter, the process noise covariance, measurement noise covariance, and initial state covariance had to be determined. The covariances were determined using predicted/observed uncertainties in the sensor readings, and the initial state variance was given by the expected variance in the robot's initial position when it was being positioned for a trial run. For the sake of the lab, additional process noise was added manually to our calculations since the TurtleBot's odometry system alone would have been sufficient to localize the robot for our purposes.

The localization was tested by having the robot stop itself when it determined that it had traveled a certain distance. During each test run, we could observe the Turtleot's behaviour and assess whether the TurtleBot had stopped close enough to the requested destination. After some iteration, the Turtlebot was succesfully able to stop within a few centimeters of any specified destination along the line.
