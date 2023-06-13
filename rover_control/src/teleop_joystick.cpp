#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "rover_control_msgs/motor_cmd.h"

// Keyword for readability
#define IN
#define OUT

#define LINEAR_INPUT joy_msg->axes[m_u_INDEX_AXIS_LINEAR]
#define ANGULAR_INPUT joy_msg->axes[m_u_INDEX_AXIS_ANGULAR]
#define TANK_MODE joy_msg->buttons[m_u_INDEX_BUTTON_ENABLE_TANK_MODE]
#define TURBO_MODE joy_msg->buttons[m_u_INDEX_BUTTON_TURBO]

float map(float x, float in_min, float in_max, float out_min, float out_max);

class TeleopJoystick
{
public:
    TeleopJoystick()
    {
        m_nh = ros::NodeHandle();
        m_sub_joy = m_nh.subscribe("rover_joy", 1, &TeleopJoystick::cbJoy, this);
        m_pub_motor_cmd = m_nh.advertise<rover_control_msgs::motor_cmd>("cmd_topic_name", 1);

        m_b_control_mode_is_tank = false;

        // Optimize mapping function
        m_f_control_map_factor = (1.0f - m_nh.param<float>(ros::this_node::getName() + "/" + "smallest_radius", 0.0f));
        getParams();
    }

    void Run()
    {
        while (!ros::isShuttingDown())
        {
            ros::spinOnce();
        }

        rover_control_msgs::motor_cmd empty_msg;
        m_pub_motor_cmd.publish(empty_msg);
    }

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub_joy;
    ros::Publisher m_pub_motor_cmd;

    uint8_t m_u_INDEX_BUTTON_ENABLE;
    uint8_t m_u_INDEX_BUTTON_TURBO;
    uint8_t m_u_INDEX_BUTTON_ENABLE_TANK_MODE;
    uint8_t m_u_INDEX_AXIS_LINEAR;
    uint8_t m_u_INDEX_AXIS_ANGULAR;

    float m_f_control_map_factor;
    bool m_b_control_mode_is_tank;

    float m_f_SPEED_FACTOR_NORMAL;
    float m_f_SPEED_FACTOR_TURBO;

    // The magic happens here
    void cbJoy(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
        // msg will always be populated with zeros in constructor
        rover_control_msgs::motor_cmd msg_motor_cmd;

        if (joy_msg->buttons[m_u_INDEX_BUTTON_ENABLE])
        {

            float f_speed_left_motor = LINEAR_INPUT;
            float f_speed_right_motor = LINEAR_INPUT;

            if (!TANK_MODE)
            {
                if (ANGULAR_INPUT > 0.0f)
                {
                    f_speed_left_motor *= 1.0f - (ANGULAR_INPUT * m_f_control_map_factor);
                }
                else
                {
                    f_speed_right_motor *= 1.0f + (ANGULAR_INPUT * m_f_control_map_factor);
                }
            }
            else
            {
                f_speed_left_motor = -ANGULAR_INPUT;
                f_speed_right_motor = ANGULAR_INPUT;
            }

            if (TURBO_MODE)
            {
                f_speed_left_motor *= m_f_SPEED_FACTOR_TURBO;
                f_speed_right_motor *= m_f_SPEED_FACTOR_TURBO;
            }
            else
            {
                f_speed_left_motor *= m_f_SPEED_FACTOR_NORMAL;
                f_speed_right_motor *= m_f_SPEED_FACTOR_NORMAL;
            }

            msg_motor_cmd.front_left = f_speed_left_motor;
            msg_motor_cmd.rear_left = f_speed_left_motor;

            msg_motor_cmd.front_right = f_speed_right_motor;
            msg_motor_cmd.rear_right = f_speed_right_motor;
        }
        m_pub_motor_cmd.publish(msg_motor_cmd);
    }

    void getParams()
    {
        const std::string s_param_prefix = (ros::this_node::getName() + "/");
        m_u_INDEX_BUTTON_ENABLE = m_nh.param<int>(s_param_prefix + "button_enable", 4);
        m_u_INDEX_BUTTON_TURBO = m_nh.param<int>(s_param_prefix + "button_turbo_", 5);
        m_u_INDEX_AXIS_LINEAR = m_nh.param<int>(s_param_prefix + "axis_linear", 1);
        m_u_INDEX_AXIS_ANGULAR = m_nh.param<int>(s_param_prefix + "axis_angular", 0);
        m_u_INDEX_BUTTON_ENABLE_TANK_MODE = m_nh.param<int>(s_param_prefix + "button_enable_tank_mode", 2);
        m_f_SPEED_FACTOR_NORMAL = m_nh.param<float>(s_param_prefix + "speed_factor_normal", 0.5);
        m_f_SPEED_FACTOR_TURBO = m_nh.param<float>(s_param_prefix + "speed_factor_turbo", 1.0);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "teleop_joystick");

    TeleopJoystick teleop_joystick = TeleopJoystick();
    teleop_joystick.Run();
    return 0;
}
