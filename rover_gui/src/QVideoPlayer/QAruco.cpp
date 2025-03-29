#include "QAruco.hpp"

QAruco::QAruco(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_)
{

    _timer_detectionManagerUpdate = _node->create_wall_timer(std::chrono::milliseconds(DELAY_DETECTION_MANAGER_UPDATE),
    [this](void)
    {
        this->CB_updateDetectionManager();
    });

    _client_ArucoDetectionManager = _node->create_client<rover_msgs::srv::ArucoDetection>(
       "/rover/auxiliary/aruco/manager");

    _ui.setupUi(this);

    connect(_ui.arucoPushButton, &QPushButton::clicked, this, &QAruco::handleDetection);

    connect(&_playerWorkerThread, &QPlayerWorker::detectionHandledSuccessfully, this, &QAruco::onDetectionHandledSuccessfully);

    connect(&_playerWorkerThread, &QPlayerWorker::urlFoundInDetection, this, &QAruco::onUrlFoundInDetection);


    _playerWorkerThread.start();
}

void QAruco::startDetection()
{
    _playerWorkerThread.manageDetection(_client_ArucoDetectionManager, _camURL,true);
}

void QAruco::stopDetection()
{
    _playerWorkerThread.manageDetection(_client_ArucoDetectionManager, _camURL,false);
}

void QAruco::handleDetection()
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

void QAruco::CB_updateDetectionManager()
{
    _playerWorkerThread.updateDetectionManager(this->_client_ArucoDetectionManager,_camURL);
}
void QAruco::onDetectionHandledSuccessfully(bool success) 
{
}

void QAruco::onUrlFoundInDetection(bool was_found_)
{
    if(!was_found_)
    {
        _ui.arucoPushButton->setChecked(false); 
    }
}