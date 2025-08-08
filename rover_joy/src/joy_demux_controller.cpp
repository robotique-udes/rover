#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/detail/joy_demux_status__struct.hpp>
#include <rover_msgs/msg/joy.hpp>
#include <rover_msgs/msg/joy_demux_status.hpp>
#include <rover_msgs/srv/joy_demux_set_state.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>
#include <utility>
#include <memory>

class JoyDemuxController : public rclcpp::Node
{
    enum eControllerType : uint8_t
    {
        MAIN = rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_MAIN,
        SECONDARY = rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_SECONDARY,
        eLAST
    };

    enum eDemuxDestination : uint8_t
    {
        // Only toggle between those two by design
        DRIVE_TRAIN = rover_msgs::srv::JoyDemuxSetState_Request::DEST_DRIVE_TRAIN,
        ARM = rover_msgs::srv::JoyDemuxSetState_Request::DEST_ARM,
        NONE = rover_msgs::srv::JoyDemuxSetState_Request::DEST_NONE,
    };

  public:
    JoyDemuxController():
        rclcpp::Node("JoyDemuxController")
    {
        _sub_demuxStatus
            = this->create_subscription<rover_msgs::msg::JoyDemuxStatus>("/base/joy/demux_status",
                                                                         QOS_DEFAULT,
                                                                         [this](const rover_msgs::msg::JoyDemuxStatus msg_)
                                                                         {
                                                                             this->CB_demuxStatus(msg_);
                                                                         });

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
            eDemuxDestination currentDest = eDemuxDestination::NONE;
            if constexpr (controller_ == eControllerType::MAIN)
            {
                currentDest = _mainControllerCurrentDestination;
            }
            else if constexpr (controller_ == eControllerType::SECONDARY)
            {
                currentDest = _secondaryControllerCurrentDestination;
            }
            else
            {
                // Not concerned
                return;
            }

            if (currentDest == eDemuxDestination::ARM)
            {
                currentDest = eDemuxDestination::DRIVE_TRAIN;
            }
            else if (currentDest == eDemuxDestination::DRIVE_TRAIN)
            {
                currentDest = eDemuxDestination::ARM;
            }
            else
            {
                // Not concerned
                return;
            }

            auto request = std::make_shared<rover_msgs::srv::JoyDemuxSetState::Request>();
            request->controller_type = std::to_underlying(controller_);
            request->force = false;
            request->destination = std::to_underlying(currentDest);

            auto response = std::make_shared<rover_msgs::srv::JoyDemuxSetState::Response>();
            auto future = _client_demuxSetState->async_send_request(request, );
            if (rclcpp::FutureReturnCode::TIMEOUT == rclcpp::spin_until_future_complete(this->shared_from_this(), response))
            {
                _client_demuxSetState->remove_pending_request(future);
            }
        }

        controllerLastJoyMsg = msg_;
    }

    void CB_demuxStatus(const rover_msgs::msg::JoyDemuxStatus msg_)
    {
        _mainControllerCurrentDestination = static_cast<eDemuxDestination>(msg_.controller_main_topic);
        _secondaryControllerCurrentDestination = static_cast<eDemuxDestination>(msg_.controller_secondary_topic);
    }

    rclcpp::Subscription<rover_msgs::msg::JoyDemuxStatus>::SharedPtr _sub_demuxStatus;
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_main;
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_secondary;
    rclcpp::Client<rover_msgs::srv::JoyDemuxSetState>::SharedPtr _client_demuxSetState;

    eDemuxDestination _mainControllerCurrentDestination = eDemuxDestination::NONE;
    eDemuxDestination _secondaryControllerCurrentDestination = eDemuxDestination::NONE;
    std::array<rover_msgs::msg::Joy, eControllerType::eLAST> _lastJoyMsgs;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<JoyDemuxController>());

    rclcpp::shutdown();
    return 0;
}
