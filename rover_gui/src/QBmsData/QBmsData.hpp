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

#include "QCellWidget.hpp"


class QBmsData : public QWidget
{
    Q_OBJECT

    static constexpr uint8_t ICON_DIMENSION = 55U;
    static constexpr uint16_t BATT_AMPS_ARRAY_SIZE = 20;
    static constexpr uint16_t GRAPH_DIMENSION = 800U;
    static constexpr uint16_t CELL_WIDTH = 200U;
    static constexpr uint16_t CELL_HEIGHT = 400U;
    static constexpr uint16_t AXIS_Y_DIFF = 20U;
    static constexpr int X_TIME_SCALER = 1000000000;
    static constexpr uint16_t CELL_MIN_VOLT = 3000;
    static constexpr uint16_t CELL_MAX_VOLT = 4200;
    static constexpr const char* TOPIC_BMS_DATA = "/rover/auxiliary/bms_data";
    static constexpr uint16_t CELLS_ARRAY_SIZE = 6;
    static constexpr uint16_t GRAPH_MAX_SAMPLES = 50;

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
        void onCallbackBmsData(const rover_msgs::msg::BmsData& msg_);

    public:
        void initializeWidget(void);
        void addCellVoltWidget(uint16_t cellIndex_, QGridLayout* grid_, uint16_t row_, uint16_t col_);
        void addBattAmpsWidget(void);
        void updateCellVolt(uint16_t cellIndex_, uint16_t voltValue_);
        void updateBattAmps(float amps_);

        std::shared_ptr<rclcpp::Node> _node;
        Ui::DataLogger _ui;
        rclcpp::Subscription<rover_msgs::msg::BmsData>::SharedPtr _sub_bmsData;
        QFlowLayout* _layout;
        std::unordered_map<uint16_t, sBmsDataInfos> _bmsDataTypes;
        QChartView* _chartView;
        QLineSeries* _battAmpsSeries;
        QValueAxis* _axisY;
        QValueAxis* _axisX;
        uint16_t _timeCount = 0;
        rclcpp::Time _initTime;
};

#endif