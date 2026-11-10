#include "QJoint.hpp"
#include <rover_lib2/helpers/assert.hpp>

QJoint::QJoint(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(std::move(guiNode_))
{
    assert(_node != nullptr);

    _ui.setupUi(this);
}