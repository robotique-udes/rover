#include "QScience.hpp"

#include <rover_lib2/helpers/assert.hpp>
#include <rover_lib2/helpers/constants.hpp>

QScience::QScience(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    ASSERT_COND(_node != nullptr);

    _ui.setupUi(this);

    _series1.setName("Sensor 1");
    _series2.setName("Sensor 2");
    _series3.setName("Sensor 3");

    _chart.addSeries(&_series1);
    _chart.addSeries(&_series2);
    _chart.addSeries(&_series3);
    _chart.createDefaultAxes();
    _chart.legend()->setVisible(true);

    _chartView.setChart(&_chart);
    _chartView.setParent(this);

    QValueAxis* axisX = qobject_cast<QValueAxis*>(_chart.axes(Qt::Horizontal).first());
    QValueAxis* axisY = qobject_cast<QValueAxis*>(_chart.axes(Qt::Vertical).first());
    axisX->setRange(0, MAX_POINTS_X);
    axisY->setRange(0, MAX_POINTS_Y);
    axisX->setTitleText("Time (s)");
    axisY->setTitleText("CO² (PPM)");

    _chartView.setRenderHint(QPainter::Antialiasing);

    _layout.addWidget(&_chartView);
    _ui.widget->setLayout(&_layout);

    connect(this, &QScience::sensorDataReceived, this, &QScience::appendSensorData, Qt::QueuedConnection);

    _sub_scienceStatus = _node->create_subscription<rover_msgs::msg::ScienceInfo>(
        TOPIC_SCIENCE_INFO,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::ScienceInfo& msg_)
        {
            this->updateSensorValues(msg_);
            emit this->sensorDataReceived(msg_.sensor_1, msg_.sensor_2, msg_.sensor_3);
        });

    this->connect(_ui.pb_clear, &QPushButton::clicked, this, &QScience::onClearClicked);
    this->connect(_ui.pb_save, &QPushButton::clicked, this, &QScience::onSaveClicked);
}

void QScience::updateSensorValues(const rover_msgs::msg::ScienceInfo& msg_)
{
    _sensor1.push_back(msg_.sensor_1);
    _sensor2.push_back(msg_.sensor_2);
    _sensor3.push_back(msg_.sensor_3);
}

void QScience::appendSensorData(quint16 s1_, quint16 s2_, quint16 s3_)
{
    // runs on the GUI thread via the queued connection
    _series1.append(_sampleIndex, s1_);
    _series2.append(_sampleIndex, s2_);
    _series3.append(_sampleIndex, s3_);
    ++_sampleIndex;

    if (_series1.count() > MAX_POINTS_X)
    {
        _series1.remove(0);
        _series2.remove(0);
        _series3.remove(0);
    }

    auto* axisX = qobject_cast<QValueAxis*>(_chart.axes(Qt::Horizontal).first());
    axisX->setRange(_sampleIndex - MAX_POINTS_X, _sampleIndex);
}

void QScience::onClearClicked()
{
    _sensor1.clear();
    _sensor2.clear();
    _sensor3.clear();

    _series1.clear();
    _series2.clear();
    _series3.clear();
    _sampleIndex = 0;
}

void QScience::onSaveClicked()
{
    RCLCPP_ERROR(this->_node->get_logger(), "Save clicked");
}