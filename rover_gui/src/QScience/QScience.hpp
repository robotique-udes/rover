#ifndef QSCIENCE_QSCIENCE_HPP
#define QSCIENCE_QSCIENCE_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/science_info.hpp>

// QT
#include "UI_Science.h"

#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <vector>

//QT_CHARTS_USE_NAMESPACE

class QScience : public QWidget
{
    Q_OBJECT

    static constexpr const char* TOPIC_SCIENCE_INFO = "/rover/science/info";
    static constexpr const char* SCIENCE_FOLDER_PATH = "/Science";
    static constexpr const char* SENSORS_FILE_PATH = "/sensorsData.csv";
    static constexpr int MAX_POINTS_X = 200;
    static constexpr int MAX_POINTS_Y = 10000;
    static constexpr double SAMPLING_RATE_SENSORS = 20.0; // Hz

  public:
    QScience(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

  signals:
    void sensorDataReceived(quint32 sampleIdx_, quint16 s1_, quint16 s2_, quint16 s3_);

  private:
    void updateSensorValues(const rover_msgs::msg::ScienceInfo& msg_);
    void createScienceFolder(void);
    void writeToCSV();

  private slots:
    void onClearClicked();
    void onSaveClicked();
    void onCheckboxClicked();
    void appendSensorData(quint32 sampleIdx_, quint16 s1_, quint16 s2_, quint16 s3_);

  private:

    std::shared_ptr<rclcpp::Node> _node;
    Ui::Science _ui;

    rclcpp::Subscription<rover_msgs::msg::ScienceInfo>::SharedPtr _sub_scienceStatus;

    std::vector<uint16_t> _sensor1;
    std::vector<uint16_t> _sensor2;
    std::vector<uint16_t> _sensor3;

    QChart _chart;
    QChartView _chartView;
    QLineSeries _series1;
    QLineSeries _series2;
    QLineSeries _series3;
    QVBoxLayout _layout;

    bool _dataPaused = false;

    std::string _sessionFolderPath;
};

#endif  // QSCIENCE_QSCIENCE_HPP