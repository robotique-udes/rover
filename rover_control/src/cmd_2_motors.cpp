// =============================================================================
// cmd_2_motors_node
// =============================================================================
// This node receive thru the antena a msg of type motor_cmd and splits it 
// between the 4 motor topics. It is also where the rover watchdog is located.
// If no signal comes from the hearbeat, this node will send column of zeros to
// each motors.
//
// =============================================================================
// =============================================================================

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "rover_control_msgs/motor_cmd.h"
#include "ros_talon/cmd.h"

struct stMotorsSpeed
{
    float f_FL = 0.0f;
    float f_FR = 0.0f;
    float f_RL = 0.0f;
    float f_RR = 0.0f;
};

class CMD2Motors
{
public:
    // =========================================================================
    // Constructor / Destructor
    // =========================================================================
    CMD2Motors()
    {
        m_nh = ros::NodeHandle();
        m_sub_heartbeat = m_nh.subscribe<std_msgs::Empty>("base_heartbeat", 1, &CMD2Motors::CBWatchdog, this);
        m_sub_motor_cmd = m_nh.subscribe<rover_control_msgs::motor_cmd>("cmd_motors", 1, &CMD2Motors::CBMotorCMD, this);
        m_pub_motor_FL = m_nh.advertise<ros_talon::cmd>("/ros_talon1/cmd", 1);
        m_pub_motor_FR = m_nh.advertise<ros_talon::cmd>("/ros_talon2/cmd", 1);
        m_pub_motor_RL = m_nh.advertise<ros_talon::cmd>("/ros_talon3/cmd", 1);
        m_pub_motor_RR = m_nh.advertise<ros_talon::cmd>("/ros_talon4/cmd", 1);
        m_pub_watchdog_state = m_nh.advertise<std_msgs::Bool>("rover_watchdog_is_alive", 1);
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

            // ROS_WARN("Time to beat is about: %f \tCurrent time is: %f",
            //          (t_last_time + dur_watchdog_reset_duration).toSec(),
            //          ros::Time::now().toSec());
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
    bool m_b_watchdog_is_alive = false;
    bool m_b_com_is_alive = false;

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
        msg.data = m_b_com_is_alive;

        m_b_watchdog_is_alive = m_b_com_is_alive;
        m_b_com_is_alive = false;

        m_pub_watchdog_state.publish(msg);
    }

    void CBMotorCMD(const rover_control_msgs::motor_cmdConstPtr &msg)
    {
        m_st_motors_speed.f_FL = msg->front_left * 100.0f;
        m_st_motors_speed.f_FR = msg->front_right * 100.0f;
        m_st_motors_speed.f_RL = msg->rear_left * 100.0f;
        m_st_motors_speed.f_RR = msg->rear_right * 100.0f;
    }

    void sendMotorCommands()
    {
        ros_talon::cmd cmd_motor_FL;
        ros_talon::cmd cmd_motor_FR;
        ros_talon::cmd cmd_motor_RL;
        ros_talon::cmd cmd_motor_RR;

        if (m_b_watchdog_is_alive)
        {
            cmd_motor_FL.cmd = m_st_motors_speed.f_FL;
            cmd_motor_FR.cmd = m_st_motors_speed.f_FR;
            cmd_motor_RL.cmd = m_st_motors_speed.f_RL;
            cmd_motor_RR.cmd = m_st_motors_speed.f_RR;
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
