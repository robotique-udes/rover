#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/motor_cmd.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

using std::placeholders::_1;

#define JOYSTICK_LEFT_FRONT 0
#define JOYSTICK_LEFT_SIDE 1
#define JOYSTICK_LEFT_PUSH 2
#define JOYSTICK_RIGHT_FRONT 3
#define JOYSTICK_RIGHT_SIDE 4
#define JOYSTICK_RIGHT_PUSH 5
#define CROSS_UP 6
#define CROSS_DOWN 7
#define CROSS_LEFT 8
#define CROSS_RIGHT 9
#define L1 10
#define L2 11
#define R1 12
#define R2 13
#define A 14
#define B 15
#define X 16
#define Y 17
#define EXT0 18
#define EXT1 19
#define EXT2 20

//Class definition
class Teleop : public rclcpp::Node
{
private:

    //Private members
    float SPEED_FACTOR_CRAWLER;
    float SPEED_FACTOR_NORMAL;
    float SPEED_FACTOR_TURBO;
    float SMALLEST_RADIUS;
    
    float DEADMAN_SWITCH;
    
    float LINEAR_INPUT;
    float ANGULAR_INPUT;
    
    float MODE_TANK;
    float MODE_NORMAL_ENABLE;
    float MODE_TURBO_ENABLE;

    float TOGGLE_LIGHTS;

    float control_map_factor = 1.0f - SMALLEST_RADIUS;

    bool light_state = false;
    bool last_lightButton_state = false;

void topic_callback(const rover_msgs::msg::Joy::SharedPtr msg)
{
    auto message = rover_msgs::msg::MotorCmd();

    if(msg->joy_data[DEADMAN_SWITCH])
    {
        float linear_speed = msg->joy_data[LINEAR_INPUT];
        float angular_speed = msg->joy_data[ANGULAR_INPUT];
        float tank_angular_speed = msg->joy_data[MODE_TANK];
        float speed_factor = SPEED_FACTOR_CRAWLER;
        bool current_lightButton_state = msg->joy_data[TOGGLE_LIGHTS];

        if(current_lightButton_state && !last_lightButton_state)
        {
            light_state = !light_state;
        }
        last_lightButton_state = current_lightButton_state;

        if(msg->joy_data[MODE_NORMAL_ENABLE]) 
        {
            speed_factor = SPEED_FACTOR_NORMAL;
        } 
        if(msg->joy_data[MODE_TURBO_ENABLE] > 0.5 && msg->joy_data[MODE_NORMAL_ENABLE])
        {
            speed_factor = SPEED_FACTOR_TURBO;
        }

        float f_speed_left_motor = linear_speed * speed_factor;
        float f_speed_right_motor = linear_speed * speed_factor;

        if(tank_angular_speed != 0.0f) 
        {
            f_speed_left_motor += tank_angular_speed * speed_factor;
            f_speed_right_motor -= tank_angular_speed * speed_factor;
        } 
        else 
        {
            float adjusted_factor;

            if (angular_speed > 0.0f) 
            {
                adjusted_factor = 1.0f - angular_speed * control_map_factor;
                f_speed_left_motor *= adjusted_factor < 0.01f ? 0.01f : adjusted_factor;
            } 
            else 
            {
                adjusted_factor = 1.0f + angular_speed * control_map_factor;
                f_speed_right_motor *= adjusted_factor < 0.01f ? 0.01f : adjusted_factor;
            }
        }

        message.front_left = f_speed_left_motor;
        message.rear_left = f_speed_left_motor;
        message.front_right = f_speed_right_motor;
        message.rear_right = f_speed_right_motor;
        message.toggle_lights = light_state;
    }

    _pub_teleop_in->publish(message);
}
    
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_formated;
    rclcpp::Publisher<rover_msgs::msg::MotorCmd>::SharedPtr _pub_teleop_in;

    void getParams()
    {        
        this->declare_parameter("speed_factor_crawler", 0.01);
        this->declare_parameter("speed_factor_normal", 0.25);
        this->declare_parameter("speed_factor_turbo", 1.0);
        this->declare_parameter("smallest_radius", 0.30);

        this->get_parameter("speed_factor_crawler", SPEED_FACTOR_CRAWLER);
        this->get_parameter("speed_factor_normal", SPEED_FACTOR_NORMAL);
        this->get_parameter("speed_factor_turbo", SPEED_FACTOR_TURBO);
        this->get_parameter("smallest_radius", SMALLEST_RADIUS);

        //Set config
        DEADMAN_SWITCH = L1;

        LINEAR_INPUT = JOYSTICK_LEFT_FRONT;
        ANGULAR_INPUT = JOYSTICK_LEFT_SIDE;

        MODE_TANK = JOYSTICK_RIGHT_SIDE;
        MODE_NORMAL_ENABLE = R1;
        MODE_TURBO_ENABLE = R2;        

        TOGGLE_LIGHTS = A;
    }

public:
    Teleop();
};

//Constructor
Teleop::Teleop() : Node("teleop")
{
    getParams();

    _sub_joy_formated = this->create_subscription<rover_msgs::msg::Joy>("/joy/main/formated",
                                                                         1,
                                                                         std::bind(&Teleop::topic_callback, this, std::placeholders::_1));

    _pub_teleop_in = this->create_publisher<rover_msgs::msg::MotorCmd>("/rover/drive_train/cmd/in/teleop", 1);
                                                                   
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
    
    return 0;
}