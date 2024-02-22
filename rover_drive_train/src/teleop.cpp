#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include <map>
#include "rover_msgs/msg/joy_demux_status.hpp"

using namespace std::placeholders;

enum Pilotes: uint8_t {
    phil = 0,
    alex,
    mimile_aintnoway,
    senile,
    rimoule,
    MAX
};

struct Actions {
    float* Deadman;
    float* Throttle;
    float* Turn;
    float* Backward;
    float* Tank;
    float* Fast;
    float* Normal;
    float* Crawl;
    float* Lights;
    float* Drift_lol;
};

struct PilotConfig {
    uint8_t Deadman_Active;
    uint8_t Throttle_Active;
    uint8_t Turn_Active;
    uint8_t TurnTank_Active;
    uint8_t Speed_Mode_Active;
    uint8_t Fast_Throttle;
    uint8_t Lights_Active;
    uint8_t Drift_lol_Active;
};

std::map<Pilotes, PilotConfig> ActionToJoyMapping;

void initConfig(Pilotes pilot, const PilotConfig& config)
{
    ActionToJoyMapping[pilot] = config;
}

void initConfigPilot(rover_msgs::msg::Joy)
{
    PilotConfig configPhil = {10, 0, 1, 4, 12, 13, 14, 15};
    initConfig(Pilotes::phil, configPhil);

    PilotConfig configsenile = {10, 13, 1, 4, 15, 11, 14, 6};
    initConfig(Pilotes::senile, configsenile);
    
}

class Teleop : public rclcpp::Node
{
private:
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_formatted;
    rclcpp::TimerBase::SharedPtr _timer;

    void callbackTeleop(const rover_msgs::msg::Joy &msg);
    void callbackTimer();
    void initConfig(Pilotes pilot, const PilotConfig& config);

public:
    Teleop();
};


Teleop::Teleop() : Node("teleop")
{
    _sub_joy_formatted = this->create_subscription<rover_msgs::msg::Joy>("main_formatted",
                                                                         1,
                                                                          [this](const rover_msgs::msg::Joy msg)
                                                                        {callbackTeleop(msg); });

    _timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Teleop::callbackTimer, this));

    this->declare_parameter("currentPilot", static_cast<int>(Pilotes::phil));                                                               
    this->declare_parameter<float>("speed_factor_crawler", 0.01);
    this->declare_parameter<float>("speed_factor_normal", 0.25);
    this->declare_parameter<float>("speed_factor_turbo", 1.0);
    this->declare_parameter<float>("smallest_radius", 0.30);

     initConfig(Pilotes::phil);
}

void Teleop::callbackTeleop(const rover_msgs::msg::Joy &msg)
{
    int pilotParam;
    this->get_parameter("currentPilot", pilotParam);
    
    Pilotes currentPilot = static_cast<Pilotes>(pilotParam);

    const auto& config = ActionToJoyMapping[currentPilot];

    //RCLCPP_INFO(this->get_logger(), "Deadman Button State: %d", msg.
    //RCLCPP_INFO(this->get_logger(), "Throttle Button State: %d", );
}

void Teleop::callbackTimer()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timer is active");
}

int main(int argc, char *argv[])
{
    float speed_factor_crawler;
    float speed_factor_normal;
    float speed_factor_turbo;
    float smallest_radius;
    
    Teleop::SharedPtr node = Teleop::make_shared("node_drive_train");

    speed_factor_crawler = static_cast<float>(node->get_parameter("speed_factor_crawler").as_double());
    speed_factor_normal = static_cast<float>(node->get_parameter("speed_factor_turbo").as_double());
    speed_factor_turbo = static_cast<float>(node->get_parameter("speed_factor_turbo").as_double());
    smallest_radius = static_cast<float>(node->get_parameter("smallest_radius").as_double());    

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Crawler %f", speed_factor_crawler);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "normal %f", speed_factor_normal);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "turbo %f", speed_factor_turbo);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "radius %f", smallest_radius);

    rclcpp::init(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
