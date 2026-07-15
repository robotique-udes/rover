#ifndef QBATTCHART_HPP
#define QBATTCHART_HPP

#include <rclcpp/rclcpp.hpp>

#include <QLabel>
#include <QtCharts>

class QBattChart : public QWidget
{
    Q_OBJECT

  public:
    static constexpr uint16_t GRAPH_DIMENSION = 800U;
    static constexpr int X_TIME_SCALER = 1000000000;
    static constexpr uint16_t GRAPH_MAX_SAMPLES = 50;

    explicit QBattChart(rclcpp::Time initTime_);
    void updateGraph(std::shared_ptr<rclcpp::Node> node_, float amps_);
    void setGraphSize(uint16_t width_ = GRAPH_DIMENSION, uint16_t height_ = GRAPH_DIMENSION);

  private:
    QChartView* _chartView;
    QLineSeries* _battAmpsSeries;
    QValueAxis* _axisY;
    QValueAxis* _axisX;
    uint16_t _timeCount = 0;
    rclcpp::Time _initTime;
};

#endif