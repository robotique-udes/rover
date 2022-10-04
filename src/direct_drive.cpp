/*
Node to control a TalonSRX device on percent output mode.
Internally, it subscribes to

/received_messages
/sent_messages
/ros_talon/motor_percent

The two first of type can_msgs::Frame from the socketcan_bridge
package, to talk to the Talon through CAN protocol. The last one
of type std_msgs::Int32 which corresponds to the actual output
Voltage in percentage at which the motor will be driven.

This node also publishes the following topics:

/ros_talon/current_position
/ros_talon/status

The first one publishes a message of type std_msgs::Float32 holding
the last position value read from the Talon, the last one publishes
a message of type ros_talon::Status holding relevant status variables
read from the Talon.

The following services are also provided:

/ros_talon/SetPID
/ros_talon/FindCenter

The first one takes three Float32 values as argument, corresponding to
the Kp, Ki and Kd values that will be configured on the Talon. These
values are used in the internal (to the Talon) PID control loop that
is used to keep the position stable in position servo mode.

The last one takes no arguments and simply takes care of finding the
center of the drive train when called.
*/

#include <talon_bridge/talon.h>
#include <ros/ros.h>
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros_talon");
	ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    unsigned int id = pnh.param("motor_id", 0);
    char topic_nb = pnh.param<std::string>("topic_nb", "\0").back();
    
    if (id == 0)
    {
        ROS_ERROR("motor_id not set");
        ros::shutdown();
    }

    if (topic_nb == '\0')
    {
        std::ostringstream oss;
        oss << id;
        std::string id_str = oss.str();
        if (id_str.size() == 1)
        {
            topic_nb = id_str[0];
            ROS_DEBUG_STREAM("topic_nb not set, using default value of motor_id: " << id);
        }    
        else
        {
            ROS_ERROR_STREAM("topic_nb not set, and motor_id [" << id << "] is longer than one char");
            ros::shutdown();
        }
    }
    
	talon::TalonSRX talon(&nh, static_cast<unsigned char>(topic_nb));
	talon.setup(static_cast<unsigned char>(id), modePercentOutput, topic_nb);

    ROS_INFO_STREAM("Motor [ID: " << id << "] connected! Using topic postfix [" << topic_nb << "]");

	ros::spin();
	return 0;
}
