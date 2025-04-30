#include "MasterDevice.hpp"

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
}

void MasterDevice::detachNode(void)
{
    this->rosElementClean();

    if (_rosNode)
    {
        _rosNode.reset();
    }
}
