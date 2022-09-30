# Lab 1
The task for lab 1 was to gain a basic understanding of how instruct the Turtlebot to move and measure its movements.

Movement instructions were sent to the Turtlebot by creating a 'motor' node that published to the cmd_vel topic. The Turtlebot onboard program was subscribed to the cmd_vel topic and continuously read messages. Thus, to tell the robot to move, it was necessary to publish a message to the cmd_vel topic.

The messages were of type 'Twist'. The message type includes three translational and rotational velocities, of which only two are used (due to the limitation of the Turtlbot's movements to 2 dimensions). The forward linear velocity and rotational velocity were set as desired and the time required to complete the desired motion could be found using the velocities used.