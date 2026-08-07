#include "QScience.hpp"

#include <filesystem>
#include <fstream>

#include <rover_lib2/helpers/assert.hpp>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_lib2/helpers/folders.hpp>

#include "Global/Helpers/QToastNotification/QToastNotification.hpp"
#include "Global/Helpers/QSessionFolderManager/QSessionFolderManager.hpp"

QScience::QScience(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    ASSERT_COND(_node != nullptr);

    _ui.setupUi(this);

    this->createScienceFolder();

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
    axisY->setTitleText("CO₂ (PPM)");

    _chartView.setRenderHint(QPainter::Antialiasing);

    _layout.addWidget(&_chartView);
    _ui.chartWidget->setLayout(&_layout);

    _sub_scienceStatus = _node->create_subscription<rover_msgs::msg::ScienceInfo>(
        TOPIC_SCIENCE_INFO,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::ScienceInfo& msg_)
        {
            if (!_dataPaused)
            {
                this->updateSensorValues(msg_);
                emit this->sensorDataReceived(msg_.sample_index, msg_.sensor_1, msg_.sensor_2, msg_.sensor_3);
            }
        });

    this->connect(this, &QScience::sensorDataReceived, this, &QScience::appendSensorData, Qt::QueuedConnection);
    this->connect(_ui.pb_clear, &QPushButton::clicked, this, &QScience::onClearClicked);
    this->connect(_ui.pb_save, &QPushButton::clicked, this, &QScience::onSaveClicked);
    this->connect(_ui.cb_pause, &QCheckBox::clicked, this, &QScience::onCheckboxClicked);
}

void QScience::updateSensorValues(const rover_msgs::msg::ScienceInfo& msg_)
{
    _sensor1.push_back(msg_.sensor_1);
    _sensor2.push_back(msg_.sensor_2);
    _sensor3.push_back(msg_.sensor_3);
}

void QScience::appendSensorData(quint32 sampleIdx_, quint16 s1_, quint16 s2_, quint16 s3_)
{
    double timeValue = sampleIdx_ / SAMPLING_RATE_SENSORS;
    _series1.append(timeValue, s1_);
    _series2.append(timeValue, s2_);
    _series3.append(timeValue, s3_);

    if (_series1.count() > MAX_POINTS_X)
    {
        _series1.remove(0);
        _series2.remove(0);
        _series3.remove(0);
    }

    QValueAxis* axisX = qobject_cast<QValueAxis*>(_chart.axes(Qt::Horizontal).first());
    axisX->setRange(timeValue - MAX_POINTS_X, timeValue);
}

void QScience::onClearClicked()
{
    _sensor1.clear();
    _sensor2.clear();
    _sensor3.clear();

    _series1.clear();
    _series2.clear();
    _series3.clear();
}

void QScience::onSaveClicked()
{
    this->writeToCSV();

    QHelper::QToastNotification::getInstance().notifyFromAnyThread("Saving chart data",
                                                                   "Saved sensors data at " + _sessionFolderPath
                                                                       + SENSORS_FILE_PATH,
                                                                   QHelper::QToastNotification::eNotifType::SUCCESS);
}

void QScience::onCheckboxClicked()
{
    if (_ui.cb_pause->isChecked())
    {
        this->_dataPaused = true;
    }
    else
    {
        this->_dataPaused = false;
    }
}

void QScience::createScienceFolder(void)
{
    std::optional<std::string> optionalSessionFolderPath = QSessionFolderManager::getInstance().getSessionFolderPath();
    std::string homePath;
    std::string sessionPath;

    if (optionalSessionFolderPath.has_value())
    {
        sessionPath = optionalSessionFolderPath.value();
        if (sessionPath.empty())
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Empty session folder path",
                                                                           "SessionFolderManager returned an empty path",
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
    }
    else
    {
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("No session folder found",
                                                                       "SessionFolderManager couldn't return a valid path",
                                                                       QHelper::QToastNotification::eNotifType::ERROR);
    }

    std::optional<std::string> optionalHomePath = Folders::getHome();
    if (optionalHomePath.has_value())
    {
        homePath = optionalHomePath.value();
        if (homePath.empty())
        {
            QHelper::QToastNotification::getInstance().notifyFromAnyThread("Empty home path",
                                                                           "HomePath returned an empty path",
                                                                           QHelper::QToastNotification::eNotifType::ERROR);
        }
    }
    else
    {
        QHelper::QToastNotification::getInstance().notifyFromAnyThread("No home found",
                                                                       "HomePath couldn't return a valid path",
                                                                       QHelper::QToastNotification::eNotifType::ERROR);
    }

    _sessionFolderPath = homePath + sessionPath + SCIENCE_FOLDER_PATH;
    Folders::createFolder(_sessionFolderPath);
    RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Current navigation folder path: %s", _sessionFolderPath.c_str());
}

void QScience::writeToCSV()
{
    std::string filePath = _sessionFolderPath + SENSORS_FILE_PATH;
    std::ofstream csv_file(filePath, std::ios_base::app);

    if (csv_file.is_open())
    {
        for (uint16_t i = 0; i < _sensor1.size(); i++)
        {
            csv_file << std::fixed << _sensor1[i] << "," << _sensor2[i] << "," << _sensor3[i] << std::endl;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "Appending file at path: %s", filePath.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to open. Try again.");
    }
}