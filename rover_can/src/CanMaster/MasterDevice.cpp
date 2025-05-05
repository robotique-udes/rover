#include "MasterDevice.hpp"
#include "rover_can2/constant.hpp"
#include "rover_lib2/helpers/macros.hpp"

#include <rover_lib2/helpers/assert.hpp>

void MasterDevice::attachNode(std::shared_ptr<rclcpp::Node> node_)
{
    if (node_ != _rosNode)
    {
        this->detachNode();
    }

    _rosNode = node_;
    ASSERT_COND_MSG(_rosNode, "Node can't be nullptr");

    this->rosElementInit();

    if (_rosNode)
    {
        std::vector ids = this->getManagedDevicesIds();

        for (const auto& id : ids)
        {
            RCLCPP_INFO(_rosNode->get_logger(),
                        "New can device %s(0x%u) registered",
                        RoverCan2::Constant::getCanDeviceName(id),
                        TO_UNDERLYING(id));
        }
    }
}

void MasterDevice::detachNode(void)
{
    this->rosElementClean();

    if (_rosNode)
    {
        _rosNode.reset();
    }
}
