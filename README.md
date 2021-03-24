# RBE595 - Humanoid Robotics - Project 1
Project 1 for WPI RBE595 - Humanoid Robotics class  
Making Atlas walk in a circle  
By John Dong and Andrew Euredijian

## Usage
First build the package by running this command in the project folder:  
`catkin_make`

To run the code, launch the Atlas simulation as usual:  
`roslauch ihmc_atlas_ros ihmc_atlas_scs_demo01.launch use_local_build:=true`

Make sure to source this package:  
`source devel/setup.bash`

Then you can use the following command to make the robot walk in a circle:  
`rosrun project1 walk_circle_node <radius> <# steps>`

Where:
 - radius: radius of the desired circle in meters
 - \# steps: number of steps to take to complete the circle (recommend at least 12)

The robot will then walk in a counter-clockwise circle of the given radius in the given number of steps.  

A good demonstration of this can be seen with the command:  
`rosrun project1 walk_circle_node 1 24`

## Description
The overall operation of this code revolves around the `stepAtPose` function provided by the `RobotWalker` class in the TOUGH API. `stepAtPose` allows us to command a specified foot to move to a specified pose with the options of queueing the pose in a queue managed internally by the `RobotWalker` class, waiting for a step to complete before executing a new one, and specifying a reference frame for the given pose. Our code utilizes all three of these options by enabling the use of the internal queue, enabling the wait for a step to run only once the previous step completes its movement, and setting the reference frame to the corresponding foot. For example, if the left foot was being moved, the reference frame would be set to the left foot. By setting up the motion to use the corresponding foot as the reference frame, the pose goal for each step becomes the same and the need to keep track of current position/orientation is removed.

To define the pose goal for each step, we start by taking in two arguments from the command line: circle radius and number of steps to take. Using these two parameters, we then calculate the circular path for each foot, define a step angle for how much each step should turn the body of the robot, and define the step length for how far along the circular path the robot should move. The step length and step angle is then decomposed into x, y, and quaternion values to assign to a Pose object which is then passed into the `stepAtPose` function to execute a stepping motion.
