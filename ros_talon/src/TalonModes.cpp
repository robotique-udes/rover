#include <talon_bridge/talon.h>

namespace talon
{
	void TalonSRX::setCmdVal(const ros_talon::cmd &c)
	{
		if(!_ignoreTopics){

			switch (c.cmd_mode){
				case 0: // mode percent output, cmd is in % [-100,100]
					_modeFunc = &TalonSRX::percentOutput;
					break;
				case 1: // mode speed pid, cmd is in rad/sec
					_modeFunc = &TalonSRX::speedPID;
					break;
				default:
					_modeFunc = NULL;
			}
			_cmd = c.cmd;
			
			if (_verbose)
				ROS_INFO_STREAM("MOTOR " << _motor_nb << ":\n\tSending this command -> " 
					<< _cmd << "\n\tWith mode -> " << c.cmd_mode);
		}

	}

	void TalonSRX::setCmdPercent(const std_msgs::Int32 &percent)
	{
		ros_talon::cmd c;

		//SET speed mode to percentoutput
		c.cmd_mode = 0;
		c.cmd = percent.data;
		setCmdVal(c);	
	}


	void TalonSRX::percentOutput()
	{
		//int32_t speed_cmd = (int32_t)(-_percent*1023/100);
		int32_t speed_cmd = (int32_t)(-_cmd*1023/100);
		if(speed_cmd > 1023)
			speed_cmd = 1023;
		else if(speed_cmd < -1023)
			speed_cmd = -1023;

		can_msgs::Frame f;
		f.id = CONTROL_3 | _baseArbID;
		f.dlc = 8;
		f.is_error = false;
		f.is_rtr = false;
		f.is_extended = true;
		for (int i = 0; i < 8; i++)
			f.data[i] = 0;
		f.data[0] = (unsigned char) (speed_cmd >> 16);
		f.data[1] = (unsigned char) (speed_cmd >> 8);
		f.data[2] = (unsigned char) (speed_cmd >> 0);

		_CANSender.publish(f);
	}

	void TalonSRX::speedPID()
	{
		//int32_t speed_cmd = (int32_t)(_speed);
		int32_t speed_cmd = (int32_t)(_cmd*0.1*4096/(2*PI)); // rad/sec * 0.1 sec/100ms * 4096 ticks/2*PI rad = ticks/100ms
		//std::cout << _speed << std::endl;
		//std::cout << speed_cmd << std::endl;
		
		can_msgs::Frame f;
		f.id = CONTROL_3 | _baseArbID;
		f.dlc = 8;
		f.is_error = false;
		f.is_rtr = false;
		f.is_extended = true;
		for (int i = 0; i < 8; i++)
			f.data[i] = 0;
		f.data[0] = (unsigned char) (speed_cmd >> 16);
		f.data[1] = (unsigned char) (speed_cmd >> 8);
		f.data[2] = (unsigned char) (speed_cmd >> 0);
		f.data[5] = (unsigned char) (2);
		_CANSender.publish(f);
	}



}; // namespace talon_interface


//velocity setpoint : position units(ticks) / 100ms

// Convert 500 RPM to units / 100ms. : 
// targetVelocity_UnitsPer100ms = leftYstick[-1. to 1.] * 500.0 * 4096 / 600;
// 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:

/*SERVO MODE*/
	/*
	void TalonSRX::setPos(const std_msgs::Float32 &f){
		if(!_ignoreTopics)
			_pos = -f.data; //Angle value in degrees between -450.0 and 450.0
	}

	void TalonSRX::ServoPos()
	{
		int32_t position = (int32_t)(_pos*6045/360);
		if(position > 7000)
			position = 7000;
		else if(position < -7000)
			position = -7000;

	/*PID SPEED*/
	/*
	void TalonSRX::setSpeed(const std_msgs::Float32 &f){
		if(!_ignoreTopics)
			_speed = f.data; //A speed in radian/s
	}*/
