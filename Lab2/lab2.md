# Lab 2: Pose Control

This Lab involved applying some slightly more complex motion that the first lab. The TurtleBot's motion was, for the first part of the lab, modelled with a unicycle model in which we could apply a linear and ungular velocity independently. Using a cunicyle model, to move from pose to pose, the Turtlebot's pose could be changed by applying any desired angular and linear velocities over different intervals of time that would result in the final desired pose.

For the second part of the lab, we were given non-holonomic constraints that we were to impose on the TurtleBot. Of course, the while TurtleBot being used for the labs has no non-holonomic constraints, it was useful to contemplate how pose control could be applied using different kinematic models that could be more relevant to other mobile robots. In this case, we assumed a system conforming to the bicycle model and imposed a minimum turning radius on the Turtlebot. For the simple path given in the lab, all that was required was a simple calculation to determine what combination of linear and angular velocities would yield the correct turning radius.

