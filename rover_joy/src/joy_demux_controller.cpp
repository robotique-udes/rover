#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/detail/joy__struct.hpp>
#include <rover_msgs/msg/joy.hpp>
#include <rover_msgs/msg/joy_demux_status.hpp>
#include <rover_msgs/srv/detail/joy_demux_set_state__struct.hpp>
#include <rover_msgs/srv/joy_demux_set_state.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>
#include <utility>

class JoyDemuxController : public rclcpp::Node
{
    enum eControllerType : size_t
    {
        MAIN = rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_MAIN,
        SECONDARY = rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_SECONDARY,
        eLAST
    };

  public:
    JoyDemuxController():
        rclcpp::Node("JoyDemuxController")
    {
        _sub_main = this->create_subscription<rover_msgs::msg::Joy>("/base/joy/main",
                                                                    QOS_DEFAULT,
                                                                    [this](const rover_msgs::msg::Joy& msg_)
                                                                    {
                                                                        this->CB_joy<eControllerType::MAIN>(msg_);
                                                                    });

        _sub_secondary = this->create_subscription<rover_msgs::msg::Joy>("/base/joy/main",
                                                                         QOS_DEFAULT,
                                                                         [this](const rover_msgs::msg::Joy& msg_)
                                                                         {
                                                                             this->CB_joy<eControllerType::SECONDARY>(msg_);
                                                                         });

        _client_demuxSetState = this->create_client<rover_msgs::srv::JoyDemuxSetState>("/base/joy/demux");
    }

  private:
    template<eControllerType controller_>
    void CB_joy(const rover_msgs::msg::Joy& msg_)
    {
        static_assert(std::to_underlying(controller_) >= 0 && std::to_underlying(controller_) < eControllerType::eLAST);

        constexpr Constants::Keybinds::eJoyInput TOGGLE_KEYBIND = Constants::Keybinds::JoyDemuxController::TOGGLE_BETWEEN_DEMUX;

        rover_msgs::msg::Joy& controllerLastJoyMsg = _lastJoyMsgs[std::to_underlying(controller_)];
        const float toggleInput = msg_.joy_data[std::to_underlying(TOGGLE_KEYBIND)];
        const float lastToggleInput = controllerLastJoyMsg.joy_data[std::to_underlying(TOGGLE_KEYBIND)];

        if (!floatToBool(toggleInput) && floatToBool(lastToggleInput))
        {
            _client_demuxSetState->async_send_request(rover_msgs::srv::JoyDemuxSetState::Request::SharedPtr)
            // Todo: toggle
        }

        controllerLastJoyMsg = msg_;
    }

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_main;
    std::array<rover_msgs::msg::Joy, eControllerType::eLAST> _lastJoyMsgs;

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_secondary;
    rover_msgs::msg::Joy _lastSecondaryMsg;

    rclcpp::Client<rover_msgs::srv::JoyDemuxSetState>::SharedPtr _client_demuxSetState;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<JoyDemuxController>());

    rclcpp::shutdown();
    return 0;
}
