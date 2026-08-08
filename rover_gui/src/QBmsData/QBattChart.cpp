#include "QBattChart.hpp"
#include "Global/Constant/StyleSheet.hpp"

QBattChart::QBattChart(rclcpp::Time initTime_):
    _initTime(initTime_)
{
    setStyleSheet(Constants::Style::BMS_STYLE);

    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(1, 1, 1, 1);
    layout->setSpacing(1);

    _battAmpsSeries = new QLineSeries();
    QChart* chart = new QChart();

    chart->addSeries(_battAmpsSeries);
    chart->setTitle("Battery ampere");

    _axisX = new QValueAxis();
    _axisX->setTitleText("Time [s]");

    _axisY = new QValueAxis();
    _axisY->setTitleText("Ampere [A]");
    _axisY->setRange(0, 20);

    chart->addAxis(_axisX, Qt::AlignBottom);
    chart->addAxis(_axisY, Qt::AlignLeft);

    _battAmpsSeries->attachAxis(_axisX);
    _battAmpsSeries->attachAxis(_axisY);

    _chartView = new QChartView(chart, this);
    _chartView->setRenderHint(QPainter::Antialiasing);

    layout->addWidget(_chartView);
}

void QBattChart::updateGraph(rclcpp::Time now_, float amps_)
{
    const double elapsed = (now_.nanoseconds() - _initTime.nanoseconds()) / NS_TO_SECONDS;
    _battAmpsSeries->append(elapsed, amps_ / -CENTIAMP_TO_AMP);

    if (_battAmpsSeries->count() > GRAPH_MAX_SAMPLES)
    {
        _battAmpsSeries->remove(0);
    }

    const double xMin = _battAmpsSeries->at(0).x();
    const double xMax = _battAmpsSeries->at(_battAmpsSeries->count() - 1).x() + X_OFFSET;

    _axisX->setRange(xMin, xMax);
}

void QBattChart::setGraphSize(uint16_t width_, uint16_t height_)
{
    setFixedSize(width_, height_);
    _chartView->setFixedSize(width_, height_);
}