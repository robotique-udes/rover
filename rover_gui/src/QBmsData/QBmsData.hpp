#ifndef QBMSDATA_HPP
#define QBMSDATA_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/bms_data.hpp>
#include <rover_can2/constant.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

// QT
#include "QCellWidget.hpp"
#include "QBattChart.hpp"
#include "Global/QFlowLayout.hpp"

class QBmsData : public QWidget
{
    Q_OBJECT

  public:
    QBmsData(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    void setGraphSize(uint16_t width_, uint16_t height_);
    void setCellContainerSize(uint16_t width_, uint16_t height_);

  signals:
    void callbackBmsData(rover_msgs::msg::BmsData msg_);

  private slots:
    void onCallbackBmsData(const rover_msgs::msg::BmsData& msg_);

  private:
    static constexpr const char* TOPIC_BMS_DATA = "/rover/auxiliary/bms_data";
    static constexpr size_t CELLS_ARRAY_SIZE = std::tuple_size_v<decltype(rover_msgs::msg::BmsData::cell_volt)>;
    static constexpr uint16_t CELLS_PER_ROW = 3;

    void initializeWidget(void);

    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Subscription<rover_msgs::msg::BmsData>::SharedPtr _sub_bmsData;
    QFlowLayout* _layout;
    std::array<QCellWidget*, CELLS_ARRAY_SIZE> _cells;
    QBattChart* _graph;
};

#endif