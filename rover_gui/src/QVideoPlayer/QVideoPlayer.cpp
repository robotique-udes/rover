#include "QVideoPlayer.hpp"

QVideoPlayer::QVideoPlayer(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
        QWidget(parent_),
        _node(guiNode_),
        _dashboardLayout(this),
        _videoFrameWidget(guiNode_, this)
    {

        _dashboardLayout.addWidget(&_videoFrameWidget);

        connect(_videoFrameWidget.getWorker(), &QPlayerWorker::urlFoundInDetection, this, &QVideoPlayer::onUrlFoundInDetection);

        _client_arucoDetectionManager = _node->create_client<rover_msgs::srv::ArucoDetection>(
        "/rover/auxiliary/aruco/manager");

        _videoFrameWidget.setArucoClientManager(_client_arucoDetectionManager);
        this->setLayout(&_dashboardLayout);

            _timer_detectionManagerUpdate = _node->create_wall_timer(std::chrono::milliseconds(DELAY_DETECTION_MANAGER_UPDATE),
        [this](void)
        {
            this->CB_updateDetectionManager();
        });


        // Add your dashboard widget here
    }

void QVideoPlayer::CB_updateDetectionManager()
{
    qDebug()<<"CB";
    _videoFrameWidget.getWorker()->updateDetectionManager(_client_arucoDetectionManager);
}


void QVideoPlayer::onUrlFoundInDetection(std::vector<std::string> live_url_list_)
{
    //for loop entre les widget
    bool urlFound = false;
    for(const auto& url:live_url_list_)
    {
        if (_videoFrameWidget.getCamURL() == url)
        {
            urlFound = true;
            break;
        }
    }
    _videoFrameWidget.setURLFound(urlFound);
    _videoFrameWidget.arucoStillAliveUpdate();


}