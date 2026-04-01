#include "QBmsData.hpp"

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



void QBmsData::callbackBmsData(const rover_msgs::msg::BmsData& msg)
{

}