
// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/bms_data.hpp>
#include <rover_can2/constant.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

// QT
#include <QtWidgets/QGridLayout>
#include "UI_BmsData.h"
#include "Global/QFlowLayout.hpp"
#include <QLabel>
#include <QtCharts>

class QBmsData;

class QCellWidget : public QWidget
{
    static constexpr uint16_t CELL_WIDTH = 200U;
    static constexpr uint16_t CELL_HEIGHT = 400U;

    public:
    QCellWidget(uint16_t cellIndex_, QBmsData* bmsGUI);
};