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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros_talon");
	ros::NodeHandle n;

	std::string motor_nb_str;
	motor_nb_str = ros::this_node::getName();
	unsigned char motor_nb_c = motor_nb_str.back();

	talon::TalonSRX talon(&n, motor_nb_c);
	talon.setup(2, modePercentOutput, motor_nb_c);

	ROS_WARN_STREAM("Motor ID 2 : " << motor_nb_c);
	ros::spin();
	return 0;
}
