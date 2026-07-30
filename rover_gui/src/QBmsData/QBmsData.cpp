#include "QBmsData.hpp"

// QT
#include <QtWidgets/QGridLayout>
#include <QLabel>
#include <QtCharts>

QBmsData::QBmsData(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{
    _bmsData = new QWidget(this);
    _gridLayout = new QGridLayout(this);
    _gridLayout->addWidget(_bmsData, 0, 0, 1, 2);
    _layout = new QFlowLayout(_bmsData);
    _layout->setSpacing(2);
    _layout->setContentsMargins(2, 2, 2, 2);

    _bmsData->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    this->initializeWidget();

    connect(this, &QBmsData::callbackBmsData, this, &QBmsData::onCallbackBmsData);

    _sub_bmsData = _node->create_subscription<rover_msgs::msg::BmsData>(TOPIC_BMS_DATA,
                                                                        QOS_DEFAULT,
                                                                        [this](const rover_msgs::msg::BmsData& msg)
                                                                        {
                                                                            this->callbackBmsData(msg);
                                                                        });
}

void QBmsData::initializeWidget(void)
{
    _graph = new QBattChart(_node->now());

    QWidget* cellContainer = new QWidget(_bmsData);
    QGridLayout* cellsGrid = new QGridLayout(cellContainer);
    cellsGrid->setSpacing(2);
    cellsGrid->setContentsMargins(2, 2, 2, 2);

    for (size_t i = 0; i < _cells.size(); i++)
    {
        _cells[i] = new QCellWidget(i);
        cellsGrid->addWidget(_cells[i], static_cast<int>(i / CELLS_PER_ROW), static_cast<int>(i % CELLS_PER_ROW));
    }

    cellContainer->setLayout(cellsGrid);
    _layout->addWidget(_graph);
    _layout->addWidget(cellContainer);
}

void QBmsData::onCallbackBmsData(const rover_msgs::msg::BmsData& msg_)
{
    if (msg_.valid)
    {
        _graph->updateGraph(_node->now(), msg_.battery_amps);
        for (size_t i = 0; i < _cells.size(); i++)
        {
            _cells[i]->setVoltage(msg_.cell_volt[i]);
        }
    }
}

void QBmsData::setGraphSize(uint16_t width_, uint16_t height_)
{
    _graph->setGraphSize(width_, height_);
}

void QBmsData::setCellContainerSize(uint16_t width_, uint16_t height_)
{
    for (size_t i = 0; i < _cells.size(); i++)
    {
        _cells[i]->setCellContainerSize(width_, height_);
    }
}