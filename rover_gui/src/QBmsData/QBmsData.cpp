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

    this->_graphXAxis.push_back(_node->now());

    this->initializeWidget();

    this->_sub_bmsData = _node->create_subscription<rover_msgs::msg::BmsData>("/rover/auxiliary/bms_data",
                                                                        QOS_DEFAULT,
                                                                        [this](rover_msgs::msg::BmsData msg_)
                                                                        {
                                                                            emit this->callbackBmsData(msg_);
                                                                        });
        
    
    connect(this, &QBmsData::callbackBmsData, this, &QBmsData::onCallbackBmsData);
}


void QBmsData::initializeWidget(void)
{
    this->addBattAmpsWidget(eMeasurementType::BATTERY_AMPS);

    std::unique_ptr<QGridLayout> cellsGrid = std::make_unique<QGridLayout>();
    cellsGrid->setSpacing(2);
    cellsGrid->setContentsMargins(2, 2, 2, 2);

    this->addCellVoltWidget(eMeasurementType::CELL_1_VOLT, cellsGrid.get(), 0, 0);
    this->addCellVoltWidget(eMeasurementType::CELL_2_VOLT, cellsGrid.get(), 0, 1);
    this->addCellVoltWidget(eMeasurementType::CELL_3_VOLT, cellsGrid.get(), 0, 2);
    this->addCellVoltWidget(eMeasurementType::CELL_4_VOLT, cellsGrid.get(), 1, 0);
    this->addCellVoltWidget(eMeasurementType::CELL_5_VOLT, cellsGrid.get(), 1, 1);
    this->addCellVoltWidget(eMeasurementType::CELL_6_VOLT, cellsGrid.get(), 1, 2);

    std::unique_ptr<QWidget> cellContainer = std::make_unique<QWidget>(this->_ui.bmsData);
    cellContainer->setLayout(cellsGrid.release());
    this->_layout->addWidget(cellContainer.release());
}

void QBmsData::addCellVoltWidget(eMeasurementType measurementType_, QGridLayout* grid_, uint16_t row_, uint16_t col_)
{
    std::unique_ptr<QWidget> bmsDataContainer = std::make_unique<QWidget>(this->_ui.bmsData);
    bmsDataContainer->setStyleSheet(DEFAULT);
    bmsDataContainer->setFixedSize(CELL_WIDTH, CELL_HEIGHT);

    std::unique_ptr<QVBoxLayout> containerLayout = std::make_unique<QVBoxLayout>(bmsDataContainer.get());
    containerLayout->setContentsMargins(1, 1, 1, 1);
    containerLayout->setSpacing(1);

    std::unique_ptr<QHBoxLayout> contentLayout = std::make_unique<QHBoxLayout>();
    contentLayout->setContentsMargins(1, 1, 1, 1);
    contentLayout->setSpacing(1);

    std::unique_ptr<QProgressBar> progressBar = std::make_unique<QProgressBar>(bmsDataContainer.get());
    progressBar->setRange(0, 4000);
    progressBar->setValue(0);
    progressBar->setTextVisible(true);
    progressBar->setAlignment(Qt::AlignCenter);

    std::unique_ptr<QLabel> titleLabel = std::make_unique<QLabel>(bmsDataContainer.get());
    titleLabel->setText(QString::fromStdString(this->getBmsDataName(measurementType_)));
    titleLabel->setAlignment(Qt::AlignCenter);

    QFont titleFont;
    titleFont.setFamily("Rajdhani");
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);

    containerLayout->addLayout(contentLayout.release());
    containerLayout->addWidget(titleLabel.release());

    this->_bmsDataTypes[measurementType_] = {bmsDataContainer.get(), progressBar.release()};

    bmsDataContainer->setLayout(containerLayout.release());
    grid_->addWidget(bmsDataContainer.release(), row_, col_);
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

    chart->addSeries(this->_battAmpsSeries);
    chart->setTitle("Ampérage de la batterie");

    std::unique_ptr<QValueAxis> axisX = std::make_unique<QValueAxis>();
    axisX->setTitleText("Temps [s]");
    axisX->setRange(0.0f, 100.0f);

    std::unique_ptr<QValueAxis> axisY = std::make_unique<QValueAxis>();
    axisY->setTitleText("Ampère [A]");
    axisY->setRange(0.0f, 1000.0f);

    chart->addAxis(axisX.get(), Qt::AlignBottom);
    chart->addAxis(axisY.get(), Qt::AlignLeft);

    this->_axisX = axisX.get();
    this->_axisY = axisY.get();
    this->_battAmpsSeries->attachAxis(axisX.release());
    this->_battAmpsSeries->attachAxis(axisY.release());

    for (size_t i = 0; i < _battAmpsDataArray.size(); i++)
    {
        _battAmpsSeries->append(i, _battAmpsDataArray[i]);
    }

    std::unique_ptr<QChartView> chartView = std::make_unique<QChartView>(chart.release(), bmsDataContainer.get());
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setFixedSize(GRAPH_DIMENSION, GRAPH_DIMENSION);

    this->_chartView = chartView.get();

    containerLayout->addWidget(chartView.release());

    bmsDataContainer->setLayout(containerLayout.release());
    this->_layout->addWidget(bmsDataContainer.release());
}


void QBmsData::onCallbackBmsData(rover_msgs::msg::BmsData msg_)
{
    this->updateBattAmps(msg_);
    this->updateBmsData(eMeasurementType::CELL_1_VOLT, msg_);
    this->updateBmsData(eMeasurementType::CELL_2_VOLT, msg_);
    this->updateBmsData(eMeasurementType::CELL_3_VOLT, msg_);
    this->updateBmsData(eMeasurementType::CELL_4_VOLT, msg_);
    this->updateBmsData(eMeasurementType::CELL_5_VOLT, msg_);
    this->updateBmsData(eMeasurementType::CELL_6_VOLT, msg_);
}

void QBmsData::updateBmsData(eMeasurementType measurementType_, rover_msgs::msg::BmsData msg_)
{
    QProgressBar* cellProgressBar = this->_bmsDataTypes[measurementType_].progressBar;

    switch (measurementType_)
    {
        case eMeasurementType::BATTERY_AMPS:
            break;
        case eMeasurementType::CELL_1_VOLT:
            cellProgressBar->setValue(msg_.cell_volt[0]);
            break;
        case eMeasurementType::CELL_2_VOLT:
            cellProgressBar->setValue(msg_.cell_volt[1]);
            break;
        case eMeasurementType::CELL_3_VOLT:
            cellProgressBar->setValue(msg_.cell_volt[2]);
            break;
        case eMeasurementType::CELL_4_VOLT:
            cellProgressBar->setValue(msg_.cell_volt[3]);
            break;
        case eMeasurementType::CELL_5_VOLT:
            cellProgressBar->setValue(msg_.cell_volt[4]);
            break;
        case eMeasurementType::CELL_6_VOLT:
            cellProgressBar->setValue(msg_.cell_volt[5]);
            break;
    }
}

void QBmsData::updateBattAmps(rover_msgs::msg::BmsData msg_)
{
    this->_battAmpsDataArray.pop_front();
    this->_battAmpsDataArray.push_back(msg_.battery_amps/-100);
    this->_graphXAxis.push_back(_node->now());

    this->_battAmpsSeries->clear();

    for (size_t i = 0; i < _battAmpsDataArray.size(); i++)
    {
        this->_battAmpsSeries->append(i, _battAmpsDataArray[i]);
    }

    this->_axisY->setRange(_battAmpsDataArray.back() - AXIS_Y_DIFF, _battAmpsDataArray.back() + AXIS_Y_DIFF);
    this->_axisX->setRange(0, (_graphXAxis.back().nanoseconds()-_graphXAxis.front().nanoseconds()) / X_TIME_SCALER + 3);
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
            return "Cell 1";
        case eMeasurementType::CELL_2_VOLT:
            return "Cell 2";
        case eMeasurementType::CELL_3_VOLT:
            return "Cell 3";
        case eMeasurementType::CELL_4_VOLT:
            return "Cell 4";
        case eMeasurementType::CELL_5_VOLT:
            return "Cell 5";
        case eMeasurementType::CELL_6_VOLT:
            return "Cell 6";
        default:
            return "Couldn't find BMS Data Type";
    }
}

void QBmsData::setGraphSize(uint16_t width_, uint16_t height_)
{
    this->_chartView->setFixedSize(width_, height_);
}

void QBmsData::setCellContainerSize(uint16_t width_, uint16_t height_)
{
    for (auto& [type, widget] : _bmsDataTypes)
    {
        widget.bmsDataContainer->setFixedSize(width_, height_);
    }
}