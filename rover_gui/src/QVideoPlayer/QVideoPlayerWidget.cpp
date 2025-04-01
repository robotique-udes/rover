#include "QVideoPlayerWidget.hpp"

QVideoPlayerWidget::QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
    {

    _ui.setupUi(this);

    connect(_ui.arucoPushButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleDetection);

    connect(&_playerWorkerThread, &QPlayerWorker::detectionHandledSuccessfully, this, &QVideoPlayerWidget::onDetectionHandledSuccessfully);

    _playerWorkerThread.start();


}

void QVideoPlayerWidget::startDetection()
{
    qDebug()<<"start";
    _playerWorkerThread.manageDetection(_client_arucoManager, _camURL,true);
}

void QVideoPlayerWidget::stopDetection()
{
    _playerWorkerThread.manageDetection(_client_arucoManager, _camURL,false);
}

void QVideoPlayerWidget::handleDetection()
{
    if(_ui.arucoPushButton->isChecked())
    {
        this->startDetection();
    }
    else
    {
        this->stopDetection();
    }
}

void QVideoPlayerWidget::onDetectionHandledSuccessfully(bool success) 
{
}


void QVideoPlayerWidget::arucoStillAliveUpdate()
{
    qDebug()<<"aruco still alive";
    if(!_urlFound)
    {
        _ui.arucoPushButton->setChecked(false);    
    }
}


std::string QVideoPlayerWidget::getCamURL(void)
{
    return this->_camURL;
}

void QVideoPlayerWidget::setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_)
{
    if (client_!=nullptr)
    {
        this->_client_arucoManager = client_;
        qDebug()<<"init!!";

    }

    else
    {
        qDebug()<<"init failed";
        #warning debug better
    }
}

QPlayerWorker* QVideoPlayerWidget::getWorker(void)
{
    return &_playerWorkerThread;
}

void QVideoPlayerWidget::setURLFound(bool urlFound_)
{
    this->_urlFound = urlFound_;
}

