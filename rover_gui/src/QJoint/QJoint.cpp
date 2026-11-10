#include <QJoint.hpp>

QJoint::QJoint(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(std::move(guiNode_))
{
    ASSERT_COND(_node != nullptr);

    _ui.setupUi(this);
}