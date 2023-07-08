// =============================================================================
// cmd_2_motors_node
// =============================================================================
// This node receive thru the antena a msg of type motor_cmd and splits it
// between the 4 motor topics. It is also where the rover watchdog is located.
// If no signal comes from the hearbeat or no signal from joy(cmd_motor), this
// node will send column of zeros to each motors.
//
// There's a low pass filter (moving average) on msg send to motors to create
// software acceleration to limit jerkiness from the rover.
// =============================================================================
// =============================================================================

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "rover_control_msgs/motor_cmd.h"
#include "ros_talon/cmd.h"

class MovingAverage
{
private:
    // =========================================================================
    // Private Members
    // =========================================================================
    uint8_t AVG_SIZE;
    uint8_t cursor = 0;
    double *values;
    double total = 0.0f;
    double avg = 0.0f;

public:
    // =========================================================================
    // Constructor / Destructor
    // =========================================================================
    MovingAverage(uint8_t avg_size = static_cast<uint8_t>(50u))
    {
        AVG_SIZE = avg_size;
        values = new double[AVG_SIZE];

        for (uint8_t i = 0; i < AVG_SIZE; i++)
        {
            values[i] = 0.0f;
        }
    }

    ~MovingAverage()
    {
        delete[] values;
    }

    // =========================================================================
    // Public Methods
    // =========================================================================
    double addValue(double value)
    {
        total -= values[cursor];
        values[cursor] = value;
        total += values[cursor];
        cursor = (cursor + 1) == AVG_SIZE ? 0 : (cursor + 1);
        avg = total / static_cast<double>(AVG_SIZE);
        return avg;
    };

    double getAverage() { return avg; };
};

struct stMotorsSpeed
{
    MovingAverage f_FL;
    MovingAverage f_FR;
    MovingAverage f_RL;
    MovingAverage f_RR;

    // =========================================================================
    // Constructor / Destructor
    // =========================================================================
    stMotorsSpeed(uint8_t window = static_cast<uint8_t>(10u)) : f_FL(window), f_FR(window), f_RL(window), f_RR(window)
    {
    }
};

struct stMotorsParam
{
    double f_FL = 1.0f;
    double f_FR = 1.0f;
    double f_RL = 1.0f;
    double f_RR = 1.0f;
};

class CMD2Motors
{
public:
    // =========================================================================
    // Constructor / Destructor
    // =========================================================================
    CMD2Motors() : m_st_motors_speed(static_cast<uint8_t>(m_nh.param<int>(ros::this_node::getName() + "/low_pass_115hz_window", 1)))
    {
        m_nh = ros::NodeHandle();
        m_sub_heartbeat = m_nh.subscribe<std_msgs::Empty>("base_heartbeat", 1, &CMD2Motors::CBWatchdog, this);
        m_sub_motor_cmd = m_nh.subscribe<rover_control_msgs::motor_cmd>("cmd_motors", 1, &CMD2Motors::CBMotorCMD, this);
        m_pub_motor_FL = m_nh.advertise<ros_talon::cmd>("/ros_talon1/cmd", 1);
        m_pub_motor_FR = m_nh.advertise<ros_talon::cmd>("/ros_talon2/cmd", 1);
        m_pub_motor_RL = m_nh.advertise<ros_talon::cmd>("/ros_talon3/cmd", 1);
        m_pub_motor_RR = m_nh.advertise<ros_talon::cmd>("/ros_talon4/cmd", 1);
        m_pub_watchdog_state = m_nh.advertise<std_msgs::Bool>("rover_watchdog_is_alive", 1);

        m_st_motors_dir.f_FL = (m_nh.param<int>(ros::this_node::getName() + "/reverse_FL", false) == true) ? -1.0 : 1.0;
        m_st_motors_dir.f_FR = (m_nh.param<int>(ros::this_node::getName() + "/reverse_FR", false) == true) ? -1.0 : 1.0;
        m_st_motors_dir.f_RL = (m_nh.param<int>(ros::this_node::getName() + "/reverse_RL", false) == true) ? -1.0 : 1.0;
        m_st_motors_dir.f_RR = (m_nh.param<int>(ros::this_node::getName() + "/reverse_RR", false) == true) ? -1.0 : 1.0;

        m_st_motors_strength.f_FL = (m_nh.param<double>(ros::this_node::getName() + "/strength_FL", 1.0));
        m_st_motors_strength.f_FR = (m_nh.param<double>(ros::this_node::getName() + "/strength_FR", 1.0));
        m_st_motors_strength.f_RL = (m_nh.param<double>(ros::this_node::getName() + "/strength_RL", 1.0));
        m_st_motors_strength.f_RR = (m_nh.param<double>(ros::this_node::getName() + "/strength_RR", 1.0));
    }

    // =========================================================================
    // Public Methods
    // =========================================================================
    void run()
    {
        int watchdog_frequency = m_nh.param<int>("watchdog_frequency", 1);
        ros::Time t_last_time = ros::Time::now();
        ros::Duration dur_watchdog_reset_duration(1.0f / static_cast<float>(watchdog_frequency));

        while (!ros::isShuttingDown())
        {
            ros::spinOnce();
            sendMotorCommands();

            if ((t_last_time + dur_watchdog_reset_duration) < ros::Time::now())
            {
                t_last_time = ros::Time::now();
                resetWatchdog();
            }
        }
    }

private:
    // =========================================================================
    // Private members
    // =========================================================================
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub_heartbeat;
    ros::Subscriber m_sub_motor_cmd;
    ros::Publisher m_pub_motor_FL;
    ros::Publisher m_pub_motor_FR;
    ros::Publisher m_pub_motor_RL;
    ros::Publisher m_pub_motor_RR;
    ros::Publisher m_pub_watchdog_state;
    stMotorsSpeed m_st_motors_speed;
    stMotorsParam m_st_motors_dir;
    stMotorsParam m_st_motors_strength;
    bool m_b_watchdog_is_alive = false;
    bool m_b_com_is_alive = false;
    bool m_b_joy_is_alive = false;

    // =========================================================================
    // Private methods
    // =========================================================================
    void CBWatchdog(const std_msgs::EmptyConstPtr &msg)
    {
        m_b_com_is_alive = true;
    }

    void resetWatchdog()
    {
        std_msgs::Bool msg;
        msg.data = (m_b_com_is_alive && m_b_joy_is_alive);

        m_b_watchdog_is_alive = (m_b_com_is_alive && m_b_joy_is_alive);
        m_b_com_is_alive = false;
        m_b_joy_is_alive = false;

        m_pub_watchdog_state.publish(msg);
    }

    void CBMotorCMD(const rover_control_msgs::motor_cmdConstPtr &msg)
    {
        m_b_joy_is_alive = true;
        m_st_motors_speed.f_FL.addValue(msg->front_left * 100.0f);
        m_st_motors_speed.f_FR.addValue(msg->front_right * 100.0f);
        m_st_motors_speed.f_RL.addValue(msg->rear_left * 100.0f);
        m_st_motors_speed.f_RR.addValue(msg->rear_right * 100.0f);
    }

    void sendMotorCommands()
    {
        ros_talon::cmd cmd_motor_FL;
        ros_talon::cmd cmd_motor_FR;
        ros_talon::cmd cmd_motor_RL;
        ros_talon::cmd cmd_motor_RR;

        if (m_b_watchdog_is_alive)
        {
            cmd_motor_FL.cmd = m_st_motors_strength.f_FL * m_st_motors_dir.f_FL * m_st_motors_speed.f_FL.getAverage();
            cmd_motor_FR.cmd = m_st_motors_strength.f_FR * m_st_motors_dir.f_FR * m_st_motors_speed.f_FR.getAverage();
            cmd_motor_RL.cmd = m_st_motors_strength.f_RL * m_st_motors_dir.f_RL * m_st_motors_speed.f_RL.getAverage();
            cmd_motor_RR.cmd = m_st_motors_strength.f_RR * m_st_motors_dir.f_RR * m_st_motors_speed.f_RR.getAverage();
        }
        else
        {
            //Simulate joy rate which is 115 Hz
            ros::Duration(1.0f/115.0f).sleep(); 
            cmd_motor_FL.cmd = m_st_motors_dir.f_FL * m_st_motors_speed.f_FL.addValue(0.0);
            cmd_motor_FR.cmd = m_st_motors_dir.f_FR * m_st_motors_speed.f_FR.addValue(0.0);
            cmd_motor_RL.cmd = m_st_motors_dir.f_RL * m_st_motors_speed.f_RL.addValue(0.0);
            cmd_motor_RR.cmd = m_st_motors_dir.f_RR * m_st_motors_speed.f_RR.addValue(0.0);
        }

        m_pub_motor_FL.publish(cmd_motor_FL);
        m_pub_motor_FR.publish(cmd_motor_FR);
        m_pub_motor_RL.publish(cmd_motor_RL);
        m_pub_motor_RR.publish(cmd_motor_RR);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_2_motors");

    CMD2Motors cmd_2_motors_node = CMD2Motors();
    cmd_2_motors_node.run();

    return 0;
}
