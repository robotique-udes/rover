#include "QBmsData.hpp"

constexpr const char* STATUS_DEFAULT = "QWidget {"
                                       "background-color: #3c3f41;"
                                       "border-radius: 5px;"
                                       "padding: 5px 10px;"
                                       "}";

QBmsData::QBmsData(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);

    _layout = std::make_unique<QFlowLayout>(_ui.bmsData);
    _layout->setSpacing(2);
    _layout->setContentsMargins(2, 2, 2, 2);

    _ui.bmsData->setLayout(_layout.get());

    _ui.bmsData->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    _ui.bmsData->adjustSize();

    this->initializeWidget();

    _sub_bmsData = _node->create_subscription<rover_msgs::msg::BmsData>("rover/auxiliary/bms_data",
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
    this->addMeasureWidget(eMeasurementType::BATTERY_AMPS);
    this->addMeasureWidget(eMeasurementType::CELL_1_VOLT);
    this->addMeasureWidget(eMeasurementType::CELL_2_VOLT);
    this->addMeasureWidget(eMeasurementType::CELL_3_VOLT);
    this->addMeasureWidget(eMeasurementType::CELL_4_VOLT);
    this->addMeasureWidget(eMeasurementType::CELL_5_VOLT);
    this->addMeasureWidget(eMeasurementType::CELL_6_VOLT);
}

void QBmsData::addMeasureWidget(eMeasurementType measurementType_)
{
    std::unique_ptr<QWidget> bmsDataContainer = std::make_unique<QWidget>(_ui.bmsData);
    bmsDataContainer->setStyleSheet(STATUS_DEFAULT);

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
    bmsInfoLabel ->setAlignment(Qt::AlignCenter);
    bmsInfoLabel ->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    bmsInfoLabel ->setWordWrap(false);

    QString infoText = QString::fromStdString(this->getBmsDataName(measurementType_)) + "0";

    bmsInfoLabel->setText(infoText);

    containerLayout->addWidget(iconLabel.release());
    containerLayout->addWidget(bmsInfoLabel.get());

    _bmsDataTypes[measurementType_] = {bmsDataContainer.get(), bmsInfoLabel.release()};

    bmsDataContainer->setLayout(containerLayout.release());
    _layout->addWidget(bmsDataContainer.release());

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
    QLabel* infoLabel = _bmsDataTypes[measurementType_].bmsInfoLabel;
    
    QString infoText = QString::fromStdString(this->getBmsDataName(measurementType_));

    switch (measurementType_)
    {
        case eMeasurementType::BATTERY_AMPS:
            infoText += QString::number(msg_.battery_amps) + " A";
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

std::string QBmsData::getBmsDataIcon(eMeasurementType measurementType_)
{
    switch (measurementType_)
    {
        case eMeasurementType::AH:
            [[fallthrough]];
        case eMeasurementType::BATTERY_AMPS:
            return ":/icons/amps.png";
        case eMeasurementType::BATTERY_VOLT:
            [[fallthrough]];
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
        case eMeasurementType::CHARGE_AMPS:
            [[fallthrough]];
        case eMeasurementType::CHARGE_VOLT:
            [[fallthrough]];
        case eMeasurementType::LOAD_AMPS:
            [[fallthrough]];
        case eMeasurementType::LOAD_VOLT:
            [[fallthrough]];
        case eMeasurementType::MAX_AH:
            [[fallthrough]];
        case eMeasurementType::SOC:
            [[fallthrough]];
        default:
            return ":/icons/motor.png";
    }
}

std::string QBmsData::getBmsDataName(eMeasurementType measurementType_)
{
    switch (measurementType_)
    {
        case eMeasurementType::AH:
            return "Ah";
        case eMeasurementType::BATTERY_AMPS:
            return "Battery \nAmps: ";
        case eMeasurementType::BATTERY_VOLT:
            return "Battery \nVolt: ";
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
        case eMeasurementType::CHARGE_AMPS:
            return "Charge Amps";
        case eMeasurementType::CHARGE_VOLT:
            return "Charge Volt";
        case eMeasurementType::LOAD_AMPS:
            return "Load Amps";
        case eMeasurementType::LOAD_VOLT:
            return "Load Volt";
        case eMeasurementType::MAX_AH:
            return "Max Ah";
        case eMeasurementType::SOC:
            return "State of charge";
        default:
            return "Couldn't find BMS Data Type";
    }
}