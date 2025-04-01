#include "QVideoPlayerWidget.hpp"

QVideoPlayerWidget::QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                                       QWidget* parent_,
                                       std::string url_,
                                       uint16_t tag_,
                                       std::shared_ptr<QPlayerWorker> worker_):
    QWidget(parent_),
    _node(guiNode_),
    _camURL(url_),
    _tag(tag_),
    _playerWorkerThread(worker_)
{
    _ui.setupUi(this);

    connect(_ui.arucoPushButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleDetection);
    connect(_playerWorkerThread.get(),
            &QPlayerWorker::detectionHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onDetectionHandledSuccessfully);
}

void QVideoPlayerWidget::startDetection()
{
    if(_playerWorkerThread.get()!=nullptr)
    {
        _playerWorkerThread->manageDetection(_client_arucoManager, _camURL,_tag, true);
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
}

void QVideoPlayerWidget::stopDetection()
{
    if(_playerWorkerThread.get()!=nullptr)
    {
        _playerWorkerThread->manageDetection(_client_arucoManager, _camURL,_tag, false);
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, couldn't access Video Player worker");
    }
}

void QVideoPlayerWidget::handleDetection()
{
    if (_ui.arucoPushButton->isChecked())
    {
        this->startDetection();
    }
    else
    {
        this->stopDetection();
    }
}

void QVideoPlayerWidget::onDetectionHandledSuccessfully(bool success_,uint16_t tag_) 
{
    if(!success_ && _tag==tag_)
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, request made on %s regarding aruco detection failed", _camURL.c_str());
    }
}

void QVideoPlayerWidget::arucoStillAliveUpdate(bool urlFound_)
{
    if (!urlFound_ && _ui.arucoPushButton->isChecked())
    {
        _ui.arucoPushButton->setChecked(false);
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, aruco detection on %s was killed", _camURL.c_str());
    }
}

std::string QVideoPlayerWidget::getCamURL(void)
{
    return this->_camURL;
}

void QVideoPlayerWidget::setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_)
{
    if (client_ != nullptr)
    {
        this->_client_arucoManager = client_;
    }

    else
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, couldn't access aruco detection manager client");
    }
}

