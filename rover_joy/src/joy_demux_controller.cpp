#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/joy.hpp>
#include <rover_msgs/msg/joy_demux_status.hpp>
#include <rover_msgs/srv/joy_demux_set_state.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>
#include <utility>
#include <memory>
#include <array>

class JoyDemuxController : public rclcpp::Node
{
    enum class eControllerType : uint8_t
    {
        MAIN = rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_MAIN,
        SECONDARY = rover_msgs::srv::JoyDemuxSetState_Request::CONTROLLER_SECONDARY,
        eLAST
    };

    enum class eDemuxDestination : uint8_t
    {
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
                                                                         [this](const rover_msgs::msg::JoyDemuxStatus& msg_)
                                                                         {
                                                                             this->CB_demuxStatus(msg_);
                                                                         });

        _sub_main = this->create_subscription<rover_msgs::msg::Joy>("/base/joy/main_formatted",
                                                                    QOS_DEFAULT,
                                                                    [this](const rover_msgs::msg::Joy& msg_)
                                                                    {
                                                                        this->CB_joy<eControllerType::MAIN>(msg_);
                                                                    });

        _sub_secondary = this->create_subscription<rover_msgs::msg::Joy>("/base/joy/secondary_formatted",
                                                                         QOS_DEFAULT,
                                                                         [this](const rover_msgs::msg::Joy& msg_)
                                                                         {
                                                                             this->CB_joy<eControllerType::SECONDARY>(msg_);
                                                                         });

        _client_demuxSetState = this->create_client<rover_msgs::srv::JoyDemuxSetState>("/base/joy/demux_control");
    }

  private:
    template<eControllerType controller_>
    void CB_joy(const rover_msgs::msg::Joy& msg_)
    {
        static_assert(std::to_underlying(controller_) < std::to_underlying(eControllerType::eLAST));

        constexpr auto TOGGLE_KEYBIND = Constants::Keybinds::JoyDemuxController::TOGGLE_BETWEEN_DEMUX;

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
                return;
            }

            if (!_client_demuxSetState->wait_for_service(std::chrono::milliseconds(10)))
            {
                RCLCPP_WARN(this->get_logger(), "Service not available");
                return;
            }

            auto request = std::make_shared<rover_msgs::srv::JoyDemuxSetState::Request>();
            request->controller_type = std::to_underlying(controller_);
            request->force = false;
            request->destination = std::to_underlying(currentDest);

            _client_demuxSetState->async_send_request(
                request,
                [this](rclcpp::Client<rover_msgs::srv::JoyDemuxSetState>::SharedFuture future_)
                {
                    try
                    {
                        auto response = future_.get();
                        RCLCPP_DEBUG(this->get_logger(), "Service call success: %s", response->success ? "True" : "False");
                    }
                    catch (const std::exception& e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Service call exception: %s", e.what());
                    }
                });
        }

        controllerLastJoyMsg = msg_;
    }

    void CB_demuxStatus(const rover_msgs::msg::JoyDemuxStatus& msg_)
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
    std::array<rover_msgs::msg::Joy, std::to_underlying(eControllerType::eLAST)> _lastJoyMsgs;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<JoyDemuxController>());

    rclcpp::shutdown();
    return 0;
}
