
#include "CanMaster/Devices/CameraMain.hpp"

#include <rover_can2/rover_can2.hpp>
#include <rclcpp/rclcpp.hpp>

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

class CanMasterNode : public rclcpp::Node
{
  public:
    CanMasterNode():
        Node("CanMasterNode")
    {
        _timerUpdateCan = this->create_wall_timer(std::chrono::milliseconds(1),
                                                  [this](void)
                                                  {
                                                      this->CB_updateCan();
                                                  });
    }

  private:
    void CB_updateCan(void)
    {
        if (!_nodeAttachedToDevices)
        {
            for (auto& device : _deviceArray)
            {
                if (device)
                {
                    device->attachNode(this->shared_from_this());
                }
            }
            _nodeAttachedToDevices = true;
        }

        _canManager.update();
    }

    bool _nodeAttachedToDevices = false;

    rclcpp::TimerBase::SharedPtr _timerUpdateCan;

    // CanDevices
    CameraMain cameraMain;

    RoverCan2::Drivers::DriverMock __canDriver;
    RoverCan2::Manager<RoverCan2::Drivers::DriverMock, CameraMain&> _canManager = RoverCan2::Manager(__canDriver, cameraMain);

    std::array<MasterDevice*, 1U> _deviceArray = {&cameraMain};
};

int main(int argc_, char* argv_[])
{
    rclcpp::init(argc_, argv_);
    rclcpp::spin(std::make_shared<CanMasterNode>());
    rclcpp::shutdown();

    return 0;
}
