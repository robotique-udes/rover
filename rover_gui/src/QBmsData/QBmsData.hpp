#ifndef QBMSDATA_HPP
#define QBMSDATA_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/bms_data.hpp>
#include <rover_can2/constant.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

// QT
#include <QtWidgets/QGridLayout>
#include "UI_BmsData.h"
#include "QCellWidget.hpp"
#include "QBattChart.hpp"
#include "Global/QFlowLayout.hpp"
#include <QLabel>
#include <QtCharts>

class QBmsData : public QWidget
{
    Q_OBJECT

  public:
    QBmsData(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    void setGraphSize(uint16_t width_ = GRAPH_DIMENSION, uint16_t height_ = GRAPH_DIMENSION);
    void setCellContainerSize(uint16_t width_ = CELL_WIDTH, uint16_t height_ = CELL_HEIGHT);

  signals:
    void callbackBmsData(rover_msgs::msg::BmsData msg_);

  private slots:
    void onCallbackBmsData(const rover_msgs::msg::BmsData& msg_);

  private:
    static constexpr uint16_t GRAPH_DIMENSION = 800U;
    static constexpr uint16_t CELL_WIDTH = 200U;
    static constexpr uint16_t CELL_HEIGHT = 400U;
    static constexpr const char* TOPIC_BMS_DATA = "/rover/auxiliary/bms_data";
    static constexpr size_t CELLS_ARRAY_SIZE = std::tuple_size_v<decltype(rover_msgs::msg::BmsData::cell_volt)>;

    void initializeWidget(void);

    std::shared_ptr<rclcpp::Node> _node;
    Ui::DataLogger _ui;
    rclcpp::Subscription<rover_msgs::msg::BmsData>::SharedPtr _sub_bmsData;
    QFlowLayout* _layout;
    std::array<QCellWidget*, CELLS_ARRAY_SIZE> _cells;
    QBattChart* _graph;
};

#endif