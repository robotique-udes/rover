README.txt

This rosbag contains 5 minutes of Talon data. The talon uses a fire-and-forget interface for all the status signals.
This means that the device it's constantly sending status data through the CAN interface. A high level explanation
of what information it's contained in each status frame may be found on the software reference manual, included in
the doc folder. It may also be found in CTRE's webpage.
A low level analysis of each status frame may be found on the ArbID_analysis.txt file on the doc folder. Code
implementation of the interface to get each of this signals is found on the Phoenix library files on CTRE folder.

This rosbag contains the /received_messages topic of type can_msgs::Frame included on ROS socketcan package.

Each message contains the data received through the CAN interface, which in this case corresponds to every status
frame sent by the Talon. Keep in mind that CTRE's can interface is little endian, so whenever they're putting a
frame together on the Phoenix library, it will be a 64 bit unsigned integer assembled as {bit7,bit6,...,bit1,bit0}.

If we denote a frame as can_msgs::Frame f, the information will be contained as f.data[0] -> bit0, f.data[1] -> bit1, etc.

You bay use this rosbag to develop any code that it's intented to process this status frames without the need of the whole
setup connected.