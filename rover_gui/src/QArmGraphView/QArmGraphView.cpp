#include "QArmGraphView.hpp"

QArmGraphView::QArmGraphView(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);

    this->initializeWidget();
}

void QArmGraphView::initializeWidget(void)
{
    _scene = new QGraphicsScene(_armViewWidget);
}

void QArmGraphView::oncallbackArmGraphView(const rover_msgs::msg::ArmMsg& msg_)
{

}