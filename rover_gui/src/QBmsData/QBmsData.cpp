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
    _ui.setupUi(this);

        _layout = new QFlowLayout(_ui.bmsData);
        _layout->setSpacing(2);
        _layout->setContentsMargins(2, 2, 2, 2);

    _ui.bmsData->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    _ui.bmsData->adjustSize();

    _initTime = _node->now();

    this->initializeWidget();

    connect(this, &QBmsData::callbackBmsData, this, &QBmsData::onCallbackBmsData);

    _sub_bmsData = _node->create_subscription<rover_msgs::msg::BmsData>(TOPIC_BMS_DATA,
                                                                        QOS_DEFAULT,
                                                                        [this](const rover_msgs::msg::BmsData& msg)
                                                                        {
                                                                            emit this->callbackBmsData(msg);
                                                                        });
}

void QBmsData::initializeWidget(void)
{
    this->addBattAmpsWidget();

    QWidget* cellContainer = new QWidget(_ui.bmsData);
    QGridLayout* cellsGrid = new QGridLayout(cellContainer);
    cellsGrid->setSpacing(2);
    cellsGrid->setContentsMargins(2, 2, 2, 2);

    for (size_t i = 0; i < CELLS_ARRAY_SIZE; i++)
    {
        this->addCellVoltWidget(i, cellsGrid, static_cast<int>(i / 3), 
                                                    static_cast<int>(i % 3));
    }

    cellContainer->setLayout(cellsGrid);
    _layout->addWidget(cellContainer);
}

void QBmsData::addCellVoltWidget(uint16_t cellIndex_, QGridLayout* grid_, uint16_t row_, uint16_t col_)
{
    QWidget* bmsDataContainer = new QWidget(_ui.bmsData);
    bmsDataContainer->setStyleSheet(DEFAULT);
    bmsDataContainer->setFixedSize(CELL_WIDTH, CELL_HEIGHT);

    QVBoxLayout* containerLayout = new QVBoxLayout(bmsDataContainer);
    containerLayout->setContentsMargins(1, 1, 1, 1);
    containerLayout->setSpacing(1);

    QProgressBar* progressBar = new QProgressBar();
    progressBar->setRange(CELL_MIN_VOLT, CELL_MAX_VOLT);
    progressBar->setValue(CELL_MIN_VOLT);
    progressBar->setFormat("%v(%p%)");
    progressBar->setTextVisible(true);
    progressBar->setAlignment(Qt::AlignCenter);
    progressBar->setOrientation(Qt::Vertical);
    progressBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QLabel* titleLabel = new QLabel();
    titleLabel->setText(QString::fromStdString("Cell " + std::to_string(cellIndex_)));
    titleLabel->setAlignment(Qt::AlignCenter);

    QFont titleFont;
    titleFont.setFamily("Rajdhani");
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);

    containerLayout->addWidget(progressBar);
    containerLayout->addWidget(titleLabel);
    _bmsDataTypes[cellIndex_] = {bmsDataContainer, progressBar};

    grid_->addWidget(bmsDataContainer, row_, col_);
}

void QBmsData::addBattAmpsWidget(void)
{
    QWidget* bmsDataContainer = new QWidget(_ui.bmsData);
    bmsDataContainer->setStyleSheet(DEFAULT);

    QHBoxLayout* containerLayout = new QHBoxLayout(bmsDataContainer);
    containerLayout->setContentsMargins(1, 1, 1, 1);
    containerLayout->setSpacing(1);

    _battAmpsSeries = new QLineSeries();
    QChart* chart = new QChart();

    chart->addSeries(_battAmpsSeries);
    chart->setTitle("Ampérage de la batterie");

    _axisX = new QValueAxis();
    _axisX->setTitleText("Temps [s]");
    _axisX->setRange(0.0f, 100.0f);

    _axisY = new QValueAxis();
    _axisY->setTitleText("Ampère [A]");
    _axisY->setRange(0.0f, 10.0f);

    chart->addAxis(_axisX, Qt::AlignBottom);
    chart->addAxis(_axisY, Qt::AlignLeft);

    _battAmpsSeries->attachAxis(_axisX);
    _battAmpsSeries->attachAxis(_axisY);

    _chartView = new QChartView(chart, bmsDataContainer);
    _chartView->setRenderHint(QPainter::Antialiasing);
    _chartView->setFixedSize(GRAPH_DIMENSION, GRAPH_DIMENSION);

    containerLayout->addWidget(_chartView);
    _layout->addWidget(bmsDataContainer);
}


void QBmsData::onCallbackBmsData(const rover_msgs::msg::BmsData& msg_)
{
    this->updateBattAmps(msg_.battery_amps);
    for (size_t i = 0; i < msg_.cell_volt.size(); i++)
    {
        this->updateCellVolt(i, msg_.cell_volt[i]);
    }
}

void QBmsData::updateCellVolt(uint16_t cellIndex_, uint16_t voltValue_)
{
    QProgressBar* cellProgressBar = _bmsDataTypes[cellIndex_].progressBar;

    cellProgressBar->setValue(voltValue_);
}

void QBmsData::updateBattAmps(float amp_)
{
    const double elapsed = (_node->now().nanoseconds() - _initTime.nanoseconds()) / X_TIME_SCALER;
    _battAmpsSeries->append(elapsed, amp_ / -100);

    if (_battAmpsSeries->count() > GRAPH_MAX_SAMPLES)
    {
        _battAmpsSeries->remove(0);
    }

    const double xMin = _battAmpsSeries->at(0).x();
    const double xMax = _battAmpsSeries->at(_battAmpsSeries->count() - 1).x() + 5;

    _axisX->setRange(xMin, xMax);
    _axisY->setRange(-10, 20);
    _timeCount++;
}

void QBmsData::setGraphSize(uint16_t width_, uint16_t height_)
{
    _chartView->setFixedSize(width_, height_);
}

void QBmsData::setCellContainerSize(uint16_t width_, uint16_t height_)
{
    for (auto& [type, widget] : _bmsDataTypes)
    {
        widget.bmsDataContainer->setFixedSize(width_, height_);
    }
}