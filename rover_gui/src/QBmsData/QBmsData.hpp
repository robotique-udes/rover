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
    static constexpr uint16_t CELL_WIDTH = 200U;
    static constexpr uint16_t CELL_HEIGHT = 400U;
    static constexpr uint16_t AXIS_Y_DIFF = 20;
    static constexpr int X_TIME_SCALER = 1000000000;

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
        QProgressBar* progressBar;
    };

    public:
        QBmsData(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
        void setGraphSize(uint16_t width_ = GRAPH_DIMENSION, uint16_t height_ = GRAPH_DIMENSION);
        void setCellContainerSize(uint16_t width_ = CELL_WIDTH, uint16_t height_ = CELL_HEIGHT);

    signals:
        void callbackBmsData(rover_msgs::msg::BmsData msg_);

    private slots:
        void onCallbackBmsData(rover_msgs::msg::BmsData msg_);

    private:
        void initializeWidget(void);
        void addCellVoltWidget(eMeasurementType measurementType_, QGridLayout* grid_, uint16_t row_, uint16_t col_);
        void addBattAmpsWidget(eMeasurementType measurementType_);
        std::string getBmsDataIcon(eMeasurementType measurementType_);
        std::string getBmsDataName(eMeasurementType measurementType_);
        void updateBmsData(eMeasurementType measurementType_,  rover_msgs::msg::BmsData msg_);
        void updateBattAmps(rover_msgs::msg::BmsData msg_);

        std::shared_ptr<rclcpp::Node> _node;
        rclcpp::TimerBase::SharedPtr _timer_bmsPub;
        rclcpp::TimerBase::SharedPtr _watchdog_bms;

        Ui::DataLogger _ui;
        rclcpp::Subscription<rover_msgs::msg::BmsData>::SharedPtr _sub_bmsData;
        std::unique_ptr<QFlowLayout> _layout;
        std::unordered_map<eMeasurementType, sBmsDataInfos> _bmsDataTypes;
        std::deque<float> _battAmpsDataArray = std::deque<float>(BATT_AMPS_ARRAY_SIZE, 0.0f);
        std::vector<rclcpp::Time> _graphXAxis;
        QChartView* _chartView;
        QLineSeries* _battAmpsSeries;
        QValueAxis* _axisY;
        QValueAxis* _axisX;


};

#endif