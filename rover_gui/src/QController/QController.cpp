#include "QController.hpp"

QController::QController(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_)
{
    _ui.setupUi(this);
}