#include <talon_bridge/talon.h>

namespace talon
{


    TalonSRX::TalonSRX(ros::NodeHandle& nh, ros::NodeHandle& private_nh, unsigned char motor_nb)
        : _nh{nh}, _private_nh{private_nh}
    {
        _motor_nb = motor_nb;
        _topic = "ros_talon";
        _topic.push_back(motor_nb);
    
        /*
        Publish to the /sent_messages topic.
        socketcan_bridge will take care of sending the can_msgs/Frame message through CAN.
        */
        _CANSender = nh.advertise<can_msgs::Frame>("sent_messages", 10);

        /*
        Publish to the ros_talon/current_position topic.
        This is taken care of by TalonSRX::unpackStatus3(const can_msgs::Frame &f).
        */

        _posPub = nh.advertise<std_msgs::Float32>(_topic + "/current_position", 10);

        /*
        Publish to the ros_talon/status topic.
        This is taken care of by TalonSRX::publishStatus(), and gets called every loop.
        */
          
        _statusPub = nh.advertise<ros_talon::Status>(_topic + "/status",10);

        /*
        Every CAN Frame received by the socket_can bridge gets published to the /recieved_messages
        topic. So we'll listen to that topic and process every frame according to its ID. This is
        done by TalonSRX::processCanFrame(const can_msgs::Frame &f)
        */
        _CANReceiver = nh.subscribe("received_messages", 10, &TalonSRX::processCanFrame, this);

        /*
        Advertise the PID service. When called, get the desired valeus and pass them to the Talon.
        This is done by TalonSRX::setPID(ros_talon::SetPID::Request  &req, ros_talon::SetPID::Response &res)
        This function is located at TalonCfg.cpp.
        */
        _spid = nh.advertiseService(_topic + "/SetPID", &TalonSRX::setPID, this);

        /*
        Advertise the setPID service. When called, drive the motor on percentOutput mode in the right direction
        until the center condition is met. This is done by
        TalonSRX::FindCenter(ros_talon::SetPID::Request  &req, ros_talon::SetPID::Response &res)
        This function is located at TalonRoutines.cpp.
        */
        _fcenter = nh.advertiseService(_topic + "/FindCenter", &TalonSRX::FindCenter, this);

        /*
        Run the loop at 20Hz to prevent the device from idling.
        */
        _talon_timer = nh.createTimer(ros::Duration(0.05), &TalonSRX::TalonLoop, this);
    }


    void TalonSRX::setup(unsigned char ID)
    {
        _ccwLimit = 0; // Limits initially unactive
        _cwLimit = 0;
        _center = 2; // Center on an uncertain state
        
        _ignoreTopics = 0; //Do not pause the main function or ignore topics
        _pauseFunction = 0;

        // Initialize other variables to 0
        _data32 = 0x00;
        _baseArbID = 0x00;
        _statusTemp = 0.0;
        _statusOutputCurrent = 0.0;
        _statusBusVoltage = 0.0;
        _statusClosedLoopError = 0.0;

        //device ID 0x3F is reserved for general addressing on enable frames.
        if(ID == 0x3F)
            ROS_ERROR("Using reserved ID.");

        //Build the base ArbID used in every frame.
        _baseArbID = (uint)(TALON | ID);

        // Remove any errors from a previous session. Some of those errors are persistant
        // on internal Talon memory.
        TalonSRX::ClearStickyFaults();

        //Add subscriber

        _TalonInput = _nh.subscribe(_topic + "/cmd", 10, &TalonSRX::setCmdVal, this);
        _TalonInput_all = _nh.subscribe("cmd_percent_motors", 10, &TalonSRX::setCmdPercent, this);

        //set default mode to be percent output
        _cmd = 0.0;
        _mode = 0;
        _modeFunc = &TalonSRX::percentOutput;

        // Initialize PID gains
        bool gains_exists = _private_nh.getParam("gains", _gains);

        if(!gains_exists)
            ROS_INFO_STREAM("MOTOR " << (int)ID << ": No gains set; grabbing default values");
        else
        {
            ROS_INFO_STREAM("MOTOR " << (int)ID << ": Setting up the gains -> {kp, ki, kd, kf}: {" + 
                std::to_string(_gains["kp"]) + ", " +
                std::to_string(_gains["ki"]) + ", " +
                std::to_string(_gains["kd"]) + ", " +
                std::to_string(_gains["kf"]));
        }

        setKP(_gains["kp"]); setKI(_gains["ki"]); setKD(_gains["kd"]); setKF(_gains["kf"]);


        if (!_private_nh.getParam("verbose", _verbose))
            _verbose = false;

        ROS_INFO_STREAM("MOTOR " << (int)ID << ": mode verbose : " << _verbose);

    }

    /*
    "Main" loop.
    */
    void TalonSRX::TalonLoop(const ros::TimerEvent& event)
    {
        /*
        An enable frame must be sent at least @10Hz.
        Otherwise, the device will go idle.
        */
        TalonSRX::enableFrame();

        /*
        Call the loop function corresponding to the current mode.
        Unless a recovery service has requested to pause the main
        function of the current mode.
        */
        if(_pauseFunction == 0){
            if(_modeFunc != NULL){
                (*this.*_modeFunc)();
            }
        }

        /*
        If no function has been set, default to percentOutput
        */
        else{
            TalonSRX::percentOutput();
        }

        /*
        If set, execute any recover functions, such as recovering from a limit or finding the center.
        */
        if(_recoverFunc != NULL){
            (*this.*_recoverFunc)();
        }

        //Publish the last read status values.
        TalonSRX::publishStatus();
    }

    void TalonSRX::enableFrame(){
        /*
        Create a CAN Frame and fill in the data according to CTRE specification for
        an enable frame.
        */
        can_msgs::Frame f; 
        f.id = 0x000401BF; // ID required for an enable frame.
        f.dlc = 8; // 8 data bytes.
        f.is_error = false; // message is not error.
        f.is_rtr = false; // message is not retransmission request.
        f.is_extended = true; // Use extended ID (29-bits instead of regular 11-bits).
        f.data[0] = 1; //First byte is the enable byte
        for (int i = 1; i < 8; i++) //Bytes 1 to 7 are 0.
            f.data[i] = 0;
        _CANSender.publish(f); //Publish the 
    }

    /*
    Functions unpackStatusX are used to unpack the data contained on the corresponding Frame.
    (e.g. unpackStatus1 is used to retreive information from the Frame that has ID STATUS_01).
    Each frame will contain different information. A full specification of what info is on each
    frame, may be found on the TalonSRX's Software Reference Manual.

    The bit operations performed to retreive the data from each Status Frame has been directly
    ported from. 
    */

    void TalonSRX::unpackStatus1(const can_msgs::Frame &f)
    {
        if((f.data[0] & 1) == 1) //first bit on the first byte of STATUS_01 indicates hardware failure.
            ROS_ERROR("ERROR: TALON REPORTS HARDWARE FAILURE.");

        //Get the limit signal values (CW and CCW Limits).
        _cwLimit = (unsigned char)((f.data[3] >> 6) & 1) == 0;
        _ccwLimit = (unsigned char)((f.data[3] >> 7) & 1) == 0;

        // If one of them is set, and we're not already performing a recovery operation, start one.
        if((_ccwLimit || _cwLimit) && _recoverFunc == NULL)
            TalonSRX::recoverLims();
    }

    void TalonSRX::unpackStatus2(const can_msgs::Frame &f)
    {
        //Output current is enconded on Status2
        unsigned char H = (unsigned char)(f.data[5]);
        unsigned char L = (unsigned char)(f.data[6]);
        H &= 0xFF;
        L &= 0xC0;
        int32_t raw = 0;
        raw |= H;
        raw <<= 8;
        raw |= L;
        raw >>= 6;
        _statusOutputCurrent = 0.125 * raw + 0.0;
    }

    void TalonSRX::unpackStatus3(const can_msgs::Frame &f)
    {
        //Current position and IDx pin status are contained on STATUS_03
        int32_t _data = 0;
        for (int i = 0; i <3; i++)
        {
            _data |= f.data[i];
            _data <<= 8;
        }
        _data >>= 8;
        
        if((int)((f.data[7] >> 4) & 1) == 1)
            _data *= 8;

        std_msgs::Float32 msg;
        msg.data = (float)_data*-360.0/6045.0;
        _currentPos = _data;
        _posPub.publish(msg);
        _center = (unsigned char)((f.data[7] >> 5) & 1);
    }

    void TalonSRX::unpackStatus4(const can_msgs::Frame &f)
    {
        //Device temperature and source voltage value are contained on STATUS_04.
        unsigned char L = (unsigned char)(f.data[0]);
        int32_t raw = 0;
        raw |= L;
        _statusBusVoltage = 0.05 * raw + 4.0;
        L = (unsigned char)(f.data[7]);
        L &= 0x3F;
        raw = 0;
        raw |= L;
        _statusTemp = (float)raw;
    }

    void TalonSRX::unpackStatus13(const can_msgs::Frame &f)
    {
        // STATUS_13 holds the current error value from the PID loop.
        unsigned char H = (unsigned char)(f.data[0]);
        unsigned char M = (unsigned char)(f.data[1]);
        unsigned char L = (unsigned char)(f.data[2]);
        int32_t error = 0;
        error |= H;
        error <<= 8;
        error |= M;
        error <<= 8;
        error |= L;
        error <<= 8;
        error >>= 8;
        // The encoder has 6045 ticks per revolution
        _statusClosedLoopError = (float)error*360.0/6045.0;

    }

    /*
    Process a CAN Frame received from Talon. Call the appropiate function based
    on the command_id.
    */

    void TalonSRX::processCanFrame(const can_msgs::Frame &f){
        _data32 = f.id;

        /*
        Process the received ID to check if its from a TalonSRX device.
        Note that deviced_id and device_number are different things. The
        device id is a unique identifier for a device familiy. Talon's device_id
        is equal to two. device_number is a user-defined identifier for a particular
        device within a family.

        An analysis of how a Frame ID is structured can be found on this project's github
        README page. The bit-wise operations performed here are a result of said analysis.
        */

        unsigned char device_number = (unsigned char)(_data32>>0);
        device_number = device_number<<2;
        device_number = device_number>>2;

        unsigned char device_id = (unsigned char)(_data32>>24);
        device_id = device_id&0x0F;

        uint32_t command_id = (_data32&0x0007FFC0);


        if((int) device_id == 2 && ((int)device_number == (int)_motor_nb -48) ){
            switch(command_id){
                case STATUS_01:
                    unpackStatus1(f);
                    break;
                case STATUS_02:
                    unpackStatus2(f);
                    break;
                case STATUS_03:
                    unpackStatus3(f);
                    break;
                case STATUS_04:
                    unpackStatus4(f);
                    break;
                case STATUS_13:
                    unpackStatus13(f);
                    break;
                case PARAM_SET:
                    ROS_INFO("PARAM_SET received.\n \t data[0] = %#02x.\n \t data[1] = %#02x.\n \t data[2] = %#02x.\n \t data[3] = %#02x.\n \t data[4] = %#02x.\n \t data[5] = %#02x.\n \t data[6] = %#02x.\n \t data[7] = %#02x.\n",f.data[0],f.data[1],f.data[2],f.data[3],f.data[4],f.data[5],f.data[6],f.data[7]);
                    break;

                default:
                    break;
            }
        }
        else if(f.is_error){
                ROS_ERROR("Frame received on can0 from unknown device is error. ArbID: %#010X.\n", f.id);
                for(int i = 0; i < 8; i++)
                    ROS_ERROR("\tdata[%d] = %#02x.\n",i,f.data[i]);
        }
    }

    void TalonSRX::publishStatus(){
        ros_talon::Status s;
        s.header.stamp = ros::Time::now();
        s.header.frame_id = "0";
        s.Temperature = _statusTemp;
        s.OutputCurrent = _statusOutputCurrent;
        s.BusVoltage = _statusBusVoltage;
        s.ClosedLoopError = _statusClosedLoopError;
        _statusPub.publish(s);
    }

}; // namespace talon_interface
