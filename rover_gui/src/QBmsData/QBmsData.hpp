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
#include "Global/QFlowLayout.hpp"
#include <QLabel>
#include <QtCharts>


class QBmsData : public QWidget
{
    Q_OBJECT

    static constexpr uint8_t ICON_DIMENSION = 55U;
    static constexpr uint16_t BATT_AMPS_ARRAY_SIZE = 20;
    static constexpr uint16_t GRAPH_DIMENSION = 800U;

    enum class eMeasurementType
    {
      BATTERY_AMPS,
      CELL_1_VOLT,
      CELL_2_VOLT,
      CELL_3_VOLT,
      CELL_4_VOLT,
      CELL_5_VOLT,
      CELL_6_VOLT
    };

    struct sBmsDataInfos
    {
        QWidget* bmsDataContainer;
        QLabel* bmsInfoLabel;
    };

    public:
        QBmsData(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
        void setGraphSize(uint16_t width_ = GRAPH_DIMENSION, uint16_t height_ = GRAPH_DIMENSION);

    private:
        void callbackBmsData(const rover_msgs::msg::BmsData& msg_);
        void initializeWidget(void);
        void addCellVoltWidget(eMeasurementType measurementType_);
        void addBattAmpsWidget(eMeasurementType measurementType_);
        std::string getBmsDataIcon(eMeasurementType measurementType_);
        std::string getBmsDataName(eMeasurementType measurementType_);
        void updateBmsData(eMeasurementType measurementType_, const rover_msgs::msg::BmsData& msg_);
        void updateBattAmps(const rover_msgs::msg::BmsData& msg_);

        std::shared_ptr<rclcpp::Node> _node;
        Ui::DataLogger _ui;
        rclcpp::Subscription<rover_msgs::msg::BmsData>::SharedPtr _sub_bmsData;
        std::unique_ptr<QFlowLayout> _layout;
        std::unordered_map<eMeasurementType, sBmsDataInfos> _bmsDataTypes;
        std::deque<float> _battAmpsDataArray = std::deque<float>(BATT_AMPS_ARRAY_SIZE, 0.0f);
        QChartView* _chartView;
        QLineSeries* _battAmpsSeries;


};

#endif