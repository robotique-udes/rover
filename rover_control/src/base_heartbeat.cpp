#include "ros/ros.h"
#include "std_msgs/Empty.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "base_heartbeat");
    ros::NodeHandle n;

    int heartbeat_frequency = 0;

    bool _flag = false;
    while(!n.getParam(ros::this_node::getName() + "/heartbeat_frequency", heartbeat_frequency))
    {
        if (!_flag)
        {
            ROS_WARN("Failed to get frequency param, retrying...");
            _flag = true;
        }
    }

    ROS_INFO("Starting heartbeat at %d Hz", heartbeat_frequency);

    ros::Publisher pub_heartbeat = n.advertise<std_msgs::Empty>("base_heartbeat", 1);
    ros::Rate timer(heartbeat_frequency);

    while (!ros::isShuttingDown())
    {
        ros::spinOnce();
        std_msgs::Empty msg;
        pub_heartbeat.publish(msg);
        timer.sleep();
    }

    return 0;
}
