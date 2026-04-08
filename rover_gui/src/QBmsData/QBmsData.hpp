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


class QBmsData : public QWidget
{
    Q_OBJECT

    static constexpr uint8_t ICON_DIMENSION = 55U;

    enum class eMeasurementType
    {
      BATTERY_AMPS,
      AH,
      MAX_AH,
      SOC,
      CHARGE_AMPS,
      LOAD_AMPS,
      BATTERY_VOLT,
      LOAD_VOLT,
      CHARGE_VOLT,
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


    private:
        void callbackBmsData(const rover_msgs::msg::BmsData& msg_);
        void initializeWidget(void);
        void addMeasureWidget(eMeasurementType measurementType_);
        std::string getBmsDataIcon(eMeasurementType measurementType_);
        std::string getBmsDataName(eMeasurementType measurementType_);
        void updateBmsData(eMeasurementType measurementType_, const rover_msgs::msg::BmsData& msg_);

        std::shared_ptr<rclcpp::Node> _node;
        Ui::DataLogger _ui;
        rclcpp::Subscription<rover_msgs::msg::BmsData>::SharedPtr _sub_bmsData;
        std::unique_ptr<QFlowLayout> _layout;
        std::unordered_map<eMeasurementType, sBmsDataInfos> _bmsDataTypes;


};

#endif