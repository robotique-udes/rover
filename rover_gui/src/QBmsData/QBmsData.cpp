#include "QBmsData.hpp"

QBmsData::QBmsData(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _ui.setupUi(this);

    _layout = new QFlowLayout(_ui.bmsData);
    _layout->setSpacing(2);
    _layout->setContentsMargins(2, 2, 2, 2);

    _ui.bmsData->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

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
    _graph = new QBattChart(_node->now());

    QWidget* cellContainer = new QWidget(_ui.bmsData);
    QGridLayout* cellsGrid = new QGridLayout(cellContainer);
    cellsGrid->setSpacing(2);
    cellsGrid->setContentsMargins(2, 2, 2, 2);

    for (uint16_t i = 0; i < CELLS_ARRAY_SIZE; i++)
    {
        _cells[i] = new QCellWidget(i);
        cellsGrid->addWidget(_cells[i], static_cast<int>(i / 3), static_cast<int>(i % 3));
    }

    cellContainer->setLayout(cellsGrid);
    _layout->addWidget(_graph);
    _layout->addWidget(cellContainer);
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
    _cells[cellIndex_]->setVoltage(voltValue_);
}

void QBmsData::updateBattAmps(float amps_)
{
    _graph->updateGraph(_node, amps_);
}

void QBmsData::setGraphSize(uint16_t width_, uint16_t height_)
{
    _graph->setGraphSize(width_, height_);
}

void QBmsData::setCellContainerSize(uint16_t width_, uint16_t height_)
{
    for (size_t i = 0; i < CELLS_ARRAY_SIZE; i++)
    {
        _cells[i]->setCellContainerSize(width_, height_);
    }
}