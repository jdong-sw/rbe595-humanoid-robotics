#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher reset_pub;
bool map_reset = false;

void resetMap(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (map_reset) return;

    ROS_INFO("Point Cloud received...");
    std_msgs::Empty empty_msg;
    reset_pub.publish(empty_msg);
    map_reset = true;
    ROS_INFO("Reset map.");
    ROS_INFO("Ready for planning.");
}

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "navigation_handler_node");
    ROS_INFO("Initializing Navigation Node...");
    ros::NodeHandle nh;

    // Set spindle speed
    double speed;
    nh.getParam("/navigation_handler_node/spindle_speed", speed);

    // Publish spindle speed
    ros::Publisher speed_pub = nh.advertise<std_msgs::Float64>("/multisense/set_spindle_speed", 1);
    std_msgs::Float64 speed_msg;
    speed_msg.data = speed;
    speed_pub.publish(speed_msg);
    ROS_INFO("Set spindle speed to %f", speed);

    // Subscribe to pointcloud topic to reset map
    ros::Subscriber sub = nh.subscribe("/atlas/assembled_cloud2", 1, resetMap);

    // Set up publisher for resetting map
    reset_pub = nh.advertise<std_msgs::Empty>("/atlas/reset_map", 1);
    ROS_INFO("Navigation Node initialized.");

    ros::spin();
    return 0;
}