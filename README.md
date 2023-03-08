# Robot Boundary Control using ROS 2
This project aims to create a boundary for the robot using any object (such as mounting tape), and ensure that the robot stays within the taped area. This solution builds upon the previous term's assessment in which the iRobot was following hand gestures, and eliminates the need for hand gestures.

# Requirements
* ROS 2 (version "Humble" or later)
* Any object that can be used to create a boundary (such as mounting tape)
* A compatible robot with sensors for edge detection
# Features
1. The robot stays within the taped area.
2. In case of an edge detection, the robot takes necessary action to either reverse or change direction wherever possible.
# Setup
1. Clone the repository to your local machine.
2. Install ROS 2 and all necessary dependencies.
3. Create a taped boundary for the robot using any object.
4. Connect the robot to the system and ensure that it has sensors for edge detection.
# Usage
Launch the ROS 2 node for the robot boundary control.
Start the robot and ensure that it stays within the taped area.
Observe the robot's behavior in case of an edge detection.
