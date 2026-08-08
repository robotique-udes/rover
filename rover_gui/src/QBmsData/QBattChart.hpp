#ifndef QBATTCHART_HPP
#define QBATTCHART_HPP

#include <rclcpp/rclcpp.hpp>

#include <QtCharts>

class QBattChart : public QWidget
{
    Q_OBJECT

  public:
    explicit QBattChart(rclcpp::Time initTime_);
    void updateGraph(rclcpp::Time now_, float amps_);
    void setGraphSize(uint16_t width_, uint16_t height_);

  private:
    static constexpr double NS_TO_SECONDS = 1'000'000'000.0;
    static constexpr uint16_t GRAPH_MAX_SAMPLES = 50;
    static constexpr double CENTIAMP_TO_AMP = 100;
    static constexpr uint16_t X_OFFSET = 5;

    QChartView* _chartView;
    QLineSeries* _battAmpsSeries;
    QValueAxis* _axisY;
    QValueAxis* _axisX;
    rclcpp::Time _initTime;
};

#endif