#include <talon_bridge/talon.h>

namespace talon
{
	/*
	Find the center when called. To do so, enter ignoreTopics mode (which means that any instruction received will
	be ignored until done with the FindCenter task), pause the current execution mode and enter outputPercent mode.

	*/
	bool TalonSRX::FindCenter(ros_talon::SetPID::Request  &req, ros_talon::SetPID::Response &res){
		TalonSRX::findCenter();
		return true;
	}
	void TalonSRX::findCenter(){
		_ignoreTopics = 1;
		_percent = 0; // Make sure the motor won't move at a previously stored speed.

		/*
		The pauseFunction variable is a workaround to change the current mode
		to outputPercent while finding the center, and the returning to the
		previous mode without know what it was.
		*/
		_pauseFunction = 1;

		/*
		There's a center signal coming from the encoder. That signal is wired to the Index pin
		on the Talon inputs, and its value gets read by the Talon itself, and then sent with
		the Status3 Frame. In consecuence, this value gets updated by
		TalonSRX::unpackStatus3(const can_msgs::Frame &f). This function is located at TalonSRX.cpp
		
		If _center is equal to 1, that means that the drive train is to the left of the center
		(i.e. rotated CCW, so we must turn CCW). If _center is equal to 0, the opposit is true, so we
		must rotate CW to find the center. The motor will be driven accordingly until a change in _center
		is read, meaning that we're now a little bit past the center in the opposite direction.
		*/

		if(_center == 1){
			_recoverFunc = &TalonSRX::findCenterR;
		}

		/*
		On setup, this variable gets a value of 2 to indicate that no valid reading has been received
		from the Talon yet, so we just wait until we get one. Notice that while doing this, I'm not
		calling findCenter() directly, avoiding an infinite loop. I'm just forcing the TalonLoop to
		call findCenter without a service call to attend.
		*/
		else if(_center == 2)
			_recoverFunc = &TalonSRX::findCenter;
		else{
			_recoverFunc = &TalonSRX::findCenterL;
		}
	}

	/*
	Drive the motor at MIN_SPEED until a change is _center is encountered.
	MIN_SPEED is defined at talon.h
	*/

	void TalonSRX::findCenterR()
	{
		if(_center){
			_percent = -MIN_SPEED;
		}
		else{
			_percent = 0;
			_pauseFunction = 0;
			_ignoreTopics = 0;
			_recoverFunc = NULL;
			setZero();
		}
	}

	void TalonSRX::findCenterL()
	{
		if(!_center){
			_percent = MIN_SPEED;
		}
		else{
			_percent = 0;
			_pauseFunction = 0;
			_ignoreTopics = 0;
			_recoverFunc = NULL;
			setZero();
		}
	}

	/*
	The motor also provides CWLimit and CCWLimit signals, meaning that the current position of
	the ring might put in danger the vehicle drive traing (going beyound +-450 degrees). when
	this happens, the system recovers from that forcing the steering wheel in the opposite direction
	until the limts get resetted.
	*/

	void TalonSRX::recoverLims(){
		_ignoreTopics = 1;
		_percent = 0;
		_pauseFunction = 1;
		if(_cwLimit == 1 && _ccwLimit == 0){
			_recoverFunc = &TalonSRX::recoverCW;
		}
		else if(_ccwLimit == 1 && _cwLimit == 0){
			_recoverFunc = &TalonSRX::recoverCCW;
		}
		else{
			ROS_ERROR("TALON_ERROR: BOTH LIMITS TRIGGERED. CHECK HARDWARE.");
		}
	}

	void TalonSRX::recoverCW()
	{
		if(_cwLimit){
			_percent = -MIN_SPEED;
		}
		else{
			_percent = 0;
			_pauseFunction = 0;
			_ignoreTopics = 0;
			_recoverFunc = NULL;
		}
	}

	void TalonSRX::recoverCCW()
	{
		if(_ccwLimit){
			_percent = MIN_SPEED;
		}
		else{
			_percent = 0;
			_pauseFunction = 0;
			_ignoreTopics = 0;
			_recoverFunc = NULL;
		}
	}

}; // namespace talon_interface