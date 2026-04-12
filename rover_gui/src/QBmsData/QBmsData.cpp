#include "QBmsData.hpp"

constexpr const char* DEFAULT = "QWidget {"
                                    "background-color: #3c3f41;"
                                    "border-radius: 5px;"
                                    "padding: 5px 10px;"
                                    "}";

QBmsData::QBmsData(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    this->_ui.setupUi(this);

    this->_layout = std::make_unique<QFlowLayout>(this->_ui.bmsData);
    this->_layout->setSpacing(2);
    this->_layout->setContentsMargins(2, 2, 2, 2);

    this->_ui.bmsData->setLayout(this->_layout.get());

    this->_ui.bmsData->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    this->_ui.bmsData->adjustSize();

    this->initializeWidget();

    this->_sub_bmsData = _node->create_subscription<rover_msgs::msg::BmsData>("rover/auxiliary/bms_data",
                                                                        QOS_DEFAULT,
                                                                        [this](const rover_msgs::msg::BmsData& msg)
                                                                        {
                                                                            QMetaObject::invokeMethod(
                                                                                this,
                                                                                [this, msg]()
                                                                                {
                                                                                    this->callbackBmsData(msg);
                                                                                },
                                                                                Qt::QueuedConnection);
                                                                        });

}


void QBmsData::initializeWidget(void)
{
    this->addBattAmpsWidget(eMeasurementType::BATTERY_AMPS);
    this->addCellVoltWidget(eMeasurementType::CELL_1_VOLT);
    this->addCellVoltWidget(eMeasurementType::CELL_2_VOLT);
    this->addCellVoltWidget(eMeasurementType::CELL_3_VOLT);
    this->addCellVoltWidget(eMeasurementType::CELL_4_VOLT);
    this->addCellVoltWidget(eMeasurementType::CELL_5_VOLT);
    this->addCellVoltWidget(eMeasurementType::CELL_6_VOLT);
}

void QBmsData::addCellVoltWidget(eMeasurementType measurementType_)
{
    std::unique_ptr<QWidget> bmsDataContainer = std::make_unique<QWidget>(this->_ui.bmsData);
    bmsDataContainer->setStyleSheet(DEFAULT);

    std::unique_ptr<QHBoxLayout> containerLayout = std::make_unique<QHBoxLayout>(bmsDataContainer.get());
    containerLayout->setContentsMargins(1, 1, 1, 1);
    containerLayout->setSpacing(1);

    std::unique_ptr<QLabel> iconLabel = std::make_unique<QLabel>(bmsDataContainer.get());
    iconLabel->setFixedSize(ICON_DIMENSION, ICON_DIMENSION);
    iconLabel->setScaledContents(true);
    iconLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    QPixmap defaultIcon(QString::fromStdString(this->getBmsDataIcon(measurementType_)));
    if (!defaultIcon.isNull())
    {
        QPixmap scaledIcon = defaultIcon.scaled(iconLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        iconLabel->setAlignment(Qt::AlignCenter);
        iconLabel->setPixmap(scaledIcon);
    }

    std::unique_ptr<QLabel> bmsInfoLabel = std::make_unique<QLabel>(bmsDataContainer.get());
    bmsInfoLabel->setAlignment(Qt::AlignCenter);
    bmsInfoLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    bmsInfoLabel->setWordWrap(false);

    QString infoText = QString::fromStdString(this->getBmsDataName(measurementType_)) + "0";

    bmsInfoLabel->setText(infoText);

    containerLayout->addWidget(iconLabel.release());
    containerLayout->addWidget(bmsInfoLabel.get());

    this->_bmsDataTypes[measurementType_] = {bmsDataContainer.get(), bmsInfoLabel.release()};

    bmsDataContainer->setLayout(containerLayout.release());
    this->_layout->addWidget(bmsDataContainer.release());
}

void QBmsData::addBattAmpsWidget(eMeasurementType measurementType_)
{
    std::unique_ptr<QWidget> bmsDataContainer = std::make_unique<QWidget>(this->_ui.bmsData);
    bmsDataContainer->setStyleSheet(DEFAULT);

    std::unique_ptr<QHBoxLayout> containerLayout = std::make_unique<QHBoxLayout>(bmsDataContainer.get());
    containerLayout->setContentsMargins(1, 1, 1, 1);
    containerLayout->setSpacing(1);

    this->_battAmpsSeries = new QLineSeries();

    std::unique_ptr<QChart> chart = std::make_unique<QChart>();

    for (size_t i = 0; i < _battAmpsDataArray.size(); i++)
    {
        _battAmpsSeries->append(i, _battAmpsDataArray[i]);
    }

    chart->addSeries(this->_battAmpsSeries);
    chart->createDefaultAxes();

    std::unique_ptr<QChartView> chartView = std::make_unique<QChartView>(chart.release(), bmsDataContainer.get());
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setFixedSize(GRAPH_DIMENSION, GRAPH_DIMENSION);

    this->_chartView = chartView.get();

    containerLayout->addWidget(chartView.release());

    bmsDataContainer->setLayout(containerLayout.release());
    this->_layout->addWidget(bmsDataContainer.release());
}


void QBmsData::callbackBmsData(const rover_msgs::msg::BmsData& msg_)
{
    this->updateBmsData(eMeasurementType::BATTERY_AMPS, msg_);
    this->updateBmsData(eMeasurementType::CELL_1_VOLT, msg_);
    this->updateBmsData(eMeasurementType::CELL_2_VOLT, msg_);
    this->updateBmsData(eMeasurementType::CELL_3_VOLT, msg_);
    this->updateBmsData(eMeasurementType::CELL_4_VOLT, msg_);
    this->updateBmsData(eMeasurementType::CELL_5_VOLT, msg_);
    this->updateBmsData(eMeasurementType::CELL_6_VOLT, msg_);
}

void QBmsData::updateBmsData(eMeasurementType measurementType_, const rover_msgs::msg::BmsData& msg_)
{
    QLabel* infoLabel = this->_bmsDataTypes[measurementType_].bmsInfoLabel;
    QString infoText = QString::fromStdString(this->getBmsDataName(measurementType_));

    switch (measurementType_)
    {
        case eMeasurementType::BATTERY_AMPS:
            updateBattAmps(msg_);
            break;
        case eMeasurementType::CELL_1_VOLT:
            infoText += QString::number(msg_.cell_volt[0]) + " mV";
            break;
        case eMeasurementType::CELL_2_VOLT:
            infoText += QString::number(msg_.cell_volt[1]) + " mV";
            break;
        case eMeasurementType::CELL_3_VOLT:
            infoText += QString::number(msg_.cell_volt[2]) + " mV";
            break;
        case eMeasurementType::CELL_4_VOLT:
            infoText += QString::number(msg_.cell_volt[3]) + " mV";
            break;
        case eMeasurementType::CELL_5_VOLT:
            infoText += QString::number(msg_.cell_volt[4]) + " mV";
            break;
        case eMeasurementType::CELL_6_VOLT:
            infoText += QString::number(msg_.cell_volt[5]) + " mV";
            break;
    }

    infoLabel->setText(infoText);
}

void QBmsData::updateBattAmps(const rover_msgs::msg::BmsData& msg_)
{
    _battAmpsDataArray.pop_front();
    _battAmpsDataArray.push_back(msg_.battery_amps);

    _battAmpsSeries->clear();

    for (size_t i = 0; i < _battAmpsDataArray.size(); i++)
    {
        _battAmpsSeries->append(i, _battAmpsDataArray[i]);
    }
}

std::string QBmsData::getBmsDataIcon(eMeasurementType measurementType_)
{
    switch (measurementType_)
    {
        case eMeasurementType::BATTERY_AMPS:
            return ":/icons/amps.png";
        case eMeasurementType::CELL_1_VOLT:
            [[fallthrough]];
        case eMeasurementType::CELL_2_VOLT:
            [[fallthrough]];
        case eMeasurementType::CELL_3_VOLT:
            [[fallthrough]];
        case eMeasurementType::CELL_4_VOLT:
            [[fallthrough]];
        case eMeasurementType::CELL_5_VOLT:
            [[fallthrough]];
        case eMeasurementType::CELL_6_VOLT:
            return ":/icons/cells_volt.png";
        default:
            return ":/icons/motor.png";
    }
}

std::string QBmsData::getBmsDataName(eMeasurementType measurementType_)
{
    switch (measurementType_)
    {
        case eMeasurementType::BATTERY_AMPS:
            return "Battery \nAmps: ";
        case eMeasurementType::CELL_1_VOLT:
            return "Cell 1 \nVolt: ";
        case eMeasurementType::CELL_2_VOLT:
            return "Cell 2 \nVolt: ";
        case eMeasurementType::CELL_3_VOLT:
            return "Cell 3 \nVolt: ";
        case eMeasurementType::CELL_4_VOLT:
            return "Cell 4 \nVolt: ";
        case eMeasurementType::CELL_5_VOLT:
            return "Cell 5 \nVolt: ";
        case eMeasurementType::CELL_6_VOLT:
            return "Cell 6 \nVolt: ";
        default:
            return "Couldn't find BMS Data Type";
    }
}

void QBmsData::setGraphSize(uint16_t width_, uint16_t height_)
{
    _chartView->setFixedSize(width_, height_);
}