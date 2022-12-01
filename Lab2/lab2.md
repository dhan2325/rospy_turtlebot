# Lab 2: Pose Control

This Lab involved applying some slightly more complex motion that the first lab. The TurtleBot's motion was, for the first part of the lab, modelled with a unicycle model in which we could apply a linear and ungular velocity independently. Using a cunicyle model, to move from pose to pose, the Turtlebot's pose could be changed by applying any desired angular and linear velocities over different intervals of time that would result in the final desired pose.

For the second part of the lab, we were given non-holonomic constraints that we were to impose on the TurtleBot. Of course, the while TurtleBot being used for the labs has no non-holonomic constraints, it was useful to contemplate how pose control could be applied using different kinematic models that could be more relevant to other mobile robots. In this case, we assumed a system conforming to the bicycle model and imposed a minimum turning radius on the Turtlebot. For the simple path given in the lab, all that was required was a simple calculation to determine what combination of linear and angular velocities would yield the correct turning radius. It was noted that for a more complete implementation of bicycle model movement, one would likely have to implement a more complete algorithm for computing an efficient path, such as the Dubin Path.

The final task in the lab was to consider various ways to have our TurtleBot follow a curve described by a mathematical function (for instance, a sinusoidal function). Our group considered a few different approaches (complete code is not provided for all implementations, this was largely a thinking exercise):

The most naive approach was to sample the curve at regular intervals, set those points as waypoints, and travel from waypoint to waypoint in straight segments and turn at each waypoint as necessary. This implementation was both inefficient and would not follow the curve very accurately.

Our next proposed method was to have a series of waypoints as before, and then using some control algorithm, regulate the difference between the pose at the next waypoint and the current pose to zero. This method was dependent on the waypoint being adequately close together for each curve, but be more efficient than the previous implementation.

The final proposition was to approximate each segment between waypoints with a circular arc, with a constant turning radius meaning the robot would have constant angular and linear velocities between waypoints. This approach was faced with difficulties similar to the last implementation.

