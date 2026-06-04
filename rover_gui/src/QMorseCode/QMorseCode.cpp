#include "QMorseCode.hpp"

#include <rover_lib2/helpers/assert.hpp>

QMorseCode::QMorseCode(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    ASSERT_COND(_node != nullptr);

    _ui.setupUi(this);

    this->connect(_ui.pb_dot,
                  &QPushButton::clicked,
                  this,
                  [this]()
                  {
                      RCLCPP_ERROR(this->_node->get_logger(), "Test dot");
                  });

    this->connect(_ui.pb_dash,
                  &QPushButton::clicked,
                  this,
                  [this]()
                  {
                      RCLCPP_ERROR(this->_node->get_logger(), "Test dash");
                  });
}
