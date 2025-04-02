#include "QVideoPlayerWidget.hpp"
#include <QStyle>

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

    connect(_ui.arucoPushButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handleArucoDetection);
    connect(_playerWorkerThread.get(),
            &QPlayerWorker::detectionHandledSuccessfully,
            this,
            &QVideoPlayerWidget::onDetectionHandledSuccessfully);
    connect(_ui.playPauseButton, &QPushButton::clicked, this, &QVideoPlayerWidget::handlePlayPauseButton);
    connect(_playerWorkerThread.get(),
            &QPlayerWorker::arucoServerInfoFailed,
            this,
            &QVideoPlayerWidget::onArucoServerInfoFailed);

    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
    _ui.rtspTextBox->setAlignment(Qt::AlignCenter);  

    _ui.arucoIdsTextBox->setText("Ids: ");

    for(size_t i=0; i<NBR_IDS_TO_DISPLAY; i++)
    {
        _lastIds[i] = 65535;
    }
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

void QVideoPlayerWidget::handleArucoDetection()
{
    if (_ui.arucoPushButton->isChecked())
    {
        this->startDetection();
        _ui.arucoPushButton->setProperty("class", "success");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);  
    }
    else
    {
        this->stopDetection();
        _ui.arucoPushButton->setProperty("class", "normal");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::onDetectionHandledSuccessfully(bool success_,uint16_t tag_) 
{
    if(!success_ && _tag==tag_)
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, request made on %s regarding aruco detection failed", _camURL.c_str());
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::arucoStillAliveUpdate(bool urlFound_)
{
    if (!urlFound_ && _ui.arucoPushButton->isChecked())
    {
        _ui.arucoPushButton->setChecked(false);
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, aruco detection on %s was killed", _camURL.c_str());
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
    if(urlFound_ && !_ui.arucoPushButton->isChecked())
    {
        _ui.arucoPushButton->setChecked(true);
        _ui.arucoPushButton->setProperty("class", "success");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::displayDetectedArucos(std::vector<uint16_t> ids_)
{
    uint16_t nbr_ids_detected = ids_.size();

    _ui.arucoIdsTextBox->setText("Ids: ");
    
    for (auto it = ids_.begin(); it != ids_.end(); ) 
    {
        if(*it == _lastIds[0])
        {
            it = ids_.erase(it);
            nbr_ids_detected--;
        }
        else
        {
            ++it;
        }
    }

    for(size_t i = 0; i < NBR_IDS_TO_DISPLAY; i++)
    {
        uint16_t new_pos = (NBR_IDS_TO_DISPLAY-(i+1)) + nbr_ids_detected;
        if(new_pos<NBR_IDS_TO_DISPLAY)
        {
            _lastIds[new_pos] = _lastIds[NBR_IDS_TO_DISPLAY-(i+1)];
        }

    }

    for (size_t i = 0; i < NBR_IDS_TO_DISPLAY; ++i) {
       
        if(i<nbr_ids_detected)
        {
            _lastIds[i] = ids_.at(i);
        }

        if(_lastIds[i]!=65535)
        {
            _ui.arucoIdsTextBox->setText(_ui.arucoIdsTextBox->text() + "  " + QString::number(_lastIds[i]));
        }
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
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

 void QVideoPlayerWidget::handlePlayPauseButton()
 {
    if (_ui.playPauseButton->isChecked()) {
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
    } 
    else {
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
    }
 }

void QVideoPlayerWidget::onArucoServerInfoFailed(bool success_)
{
    if(!success_)
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, info request to aruco detection manager client failed");
        if(_ui.arucoPushButton->property("class") != "error") 
        {
            _ui.arucoPushButton->setProperty("class", "warning");
            _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
            _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
        }
    }
    else
    {
        if(_ui.arucoPushButton->property("class") != "success" && _ui.arucoPushButton->property("class") != "error") 
        {
            _ui.arucoPushButton->setProperty("class", "normal");
            _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
            _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
        }
    }
}

