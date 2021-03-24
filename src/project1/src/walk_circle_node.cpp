#include <ros/ros.h>
#include <tough_footstep/robot_walker.h>
#include <geometry_msgs/Pose.h>
#include <ihmc_msgs/FootstepDataRosMessage.h>
#include <tf/tf.h>
#include <math.h>

#define PI 3.1415

int main(int argc, char **argv)
{
    // Initialize a ros node
    ROS_INFO("Initializing Node...");
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();   // This will ensure all ROS callbacks are activated

    // Initialize RobotWalker instance
    ROS_INFO("Initializing Walker...");
    RobotWalker walk(nh, 0.8, 1.2);

    // Parse cli args
    double radius = std::atof(argv[1]); // centered on COM ; meters
    int steps = std::atof(argv[2]);

    // Calculate values to define a circular path for each foot
    float foot_offset = 0.328;
	double right_radius = radius + foot_offset/2; // meters
	ROS_INFO("Right Radius: %f", right_radius);
	double left_radius = radius - foot_offset/2; // meters
	ROS_INFO("Left Radius: %f", left_radius);

	double angle_delta = (2 * PI) / steps; // radians
	ROS_INFO("Step Angle: %f", angle_delta);

	// Initialize loop vars
	ihmc_msgs::FootstepDataRosMessage left_foot, right_foot;
	geometry_msgs::Pose left_pose, right_pose;
    tf::Quaternion t_left, t_right;
    int side = 0; // Start with left foot
    float left_multiplier, right_multiplier, right_multiplier2;
	double right_stepLength, left_stepLength;

	// Loop to walk in a circle
	for (int i = 0; i < steps + 1; ++i)
    {
		// Calculate step position as offset from each foot's frame

        // Left foot (inner)
        if (i == 0 || i == steps) {
            left_multiplier = 1.0;
		} else {
            left_multiplier = 2.0;
		}

        left_stepLength = left_radius * sin(left_multiplier * angle_delta) / sin((PI - left_multiplier * angle_delta)/2);
        left_pose.position.x = left_stepLength * cos(left_multiplier * angle_delta / 2);
        left_pose.position.y = left_stepLength * sin(left_multiplier * angle_delta / 2);
		
        t_left = tf::createQuaternionFromYaw(left_multiplier * angle_delta);
        left_pose.orientation.w = t_left.w();
        left_pose.orientation.x = t_left.x();
        left_pose.orientation.y = t_left.y();
        left_pose.orientation.z = t_left.z();

        // Right foot (outer)
        if (i == 1) {
            right_multiplier = 1.0;
            right_multiplier2 = 1.0;
        } else if (i == steps) {
            right_multiplier = 2.0;
            right_multiplier2 = 0.5;
		} else {
            right_multiplier = 2.0;
            right_multiplier2 = 1.0;
		}

        right_stepLength = right_radius * sin(right_multiplier2 * 2 * angle_delta) / sin((PI - right_multiplier2 * 2 * angle_delta)/2);
        right_pose.position.x = right_stepLength * cos(right_multiplier * angle_delta);
        right_pose.position.y = right_stepLength * sin(right_multiplier * angle_delta);
        
        t_right = tf::createQuaternionFromYaw(right_multiplier * angle_delta);
        right_pose.orientation.w = t_right.w();
        right_pose.orientation.x = t_right.x();
        right_pose.orientation.y = t_right.y();
        right_pose.orientation.z = t_right.z();

		// Execute step
        if (side == 0) { // left
            ROS_INFO("LEFT %d-- (X, Y) : %f, %f", i, left_pose.position.x, left_pose.position.y);
            walk.stepAtPose(left_pose, (RobotSide)side, true, true, "/l_foot");
            side = 1;
        } else { // right		
            ROS_INFO("RIGHT %d-- (X, Y) : %f, %f", i, right_pose.position.x, right_pose.position.y);
            walk.stepAtPose(right_pose, (RobotSide)side, true, true, "/r_foot");
            side = 0;
        }
    }
}