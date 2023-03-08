# Robot Boundary Control using ROS 2
This project aims to create a boundary for the robot using any object (such as mounting tape), and ensure that the robot stays within the taped area. This solution builds upon the previous term's assessment in which the iRobot was following hand gestures, and eliminates the need for hand gestures.

# Screenshots
1. The gazebo world with the Robot launch
![Alt text](https://github.com/jinoAlgon/EdgeDetect_Gazebo/blob/main/output_snaps/My_world.jpg "my_world title")

2. Detecting edges
![Alt text](https://github.com/jinoAlgon/EdgeDetect_Gazebo/blob/main/output_snaps/Entire_image.jpg "my_world canny")

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
# Run
1. Create a workspace and Make sure you are in the src folder before running the package creation command.
  cd ~/ros2_ws/src

2. Creating a new package in ROS 2 is:
ros2 pkg create --build-type ament_cmake <package_name>

3. Clone the repository
https://github.com/jinoAlgon/EdgeDetect_Gazebo.git

4. Then go to your workspace directory and run
* source /opt/ros/humble/setup.bash
* colcon build

5. Open a new terminal and use
ros2 launch 
EdgeDetect_Gazebo launch_sim.launch.py world:=/home/jino/dev_ws/src/my_car/worlds/my_home.world

6. Open a new terminal and cd into  EdgeDetect_Gazebo/launch/

7. Enter this command
python3 new_1.py

# Screenshots

# Usage
* Launch the ROS 2 node for the robot boundary control.
* Start the robot and ensure that it stays within the taped area.
* Observe the robot's behavior in case of an edge detection.
