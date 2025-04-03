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
    this->_defaultCamUrl = _camURL;
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
    
    connect(_ui.rtspTextBox, &QLineEdit::textChanged,this, &QVideoPlayerWidget::updateCamURL);
    connect(_ui.defaultStreamPushButton, &QPushButton::clicked,this, &QVideoPlayerWidget::setURLToDefault);


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
        if (!_ui.arucoPushButton->isChecked()) 
        {
            _ui.arucoPushButton->setChecked(true);
        }
        _ui.arucoPushButton->setProperty("class", "success");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);  
    }
    else
    {
        this->stopDetection();
        if (_ui.arucoPushButton->isChecked()) 
        {
            _ui.arucoPushButton->setChecked(false);
        }        
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
        _ui.arucoPushButton->setChecked(true);
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, aruco detection on %s was not found", _camURL.c_str());
        _ui.arucoPushButton->setProperty("class", "normal");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::arucoCameraFailure(bool valid_)
{
    if (!valid_)
    {
        if (_ui.arucoPushButton->isChecked()) 
        {
            _ui.arucoPushButton->setChecked(false);
        }
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, camera at %s is not accessible", _camURL.c_str());
        _ui.arucoPushButton->setProperty("class", "error");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }

    if(valid_ && !_ui.arucoPushButton->isChecked())
    {
        if (!_ui.arucoPushButton->isChecked()) 
        {
            _ui.arucoPushButton->setChecked(true);
        }
        _ui.arucoPushButton->setProperty("class", "success");
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
}

void QVideoPlayerWidget::displayDetectedArucos(std::vector<uint16_t> ids_)
{
    uint16_t nbr_ids_detected = ids_.size();

    if(nbr_ids_detected>NBR_IDS_TO_DISPLAY)
    {
        ids_.resize(5);
    }
    _ui.arucoIdsTextBox->setText("Ids: ");
    
    for (const auto& id:ids_) 
    {
        _ui.arucoIdsTextBox->setText(_ui.arucoIdsTextBox->text() + "  " + QString::number(id));
    }

}


std::string QVideoPlayerWidget::getCamURL(void)
{
    return this->_camURL;
}

void QVideoPlayerWidget::updateCamURL()
{
    _camURL = _ui.rtspTextBox->text().toStdString();
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
    if (_ui.playPauseButton->isChecked()) 
    {
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-start"));
    }
    else 
    {
        _ui.playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause"));
    }
}

void QVideoPlayerWidget::setURLToDefault()
{
    _camURL = this->_defaultCamUrl;
    _ui.rtspTextBox->setText(QString::fromStdString(_camURL));
}

void QVideoPlayerWidget::setCamURL(std::string _newCamUrl)
{
    _camURL = _newCamUrl;
}

void QVideoPlayerWidget::onArucoServerInfoFailed(bool success_)
{
    if(!success_)
    {
        RCLCPP_WARN(rclcpp::get_logger("GUI"), "Error, info request to aruco detection manager client failed");
        _ui.arucoPushButton->setProperty("class", "disabled");
        _ui.arucoPushButton->setEnabled(false);
        _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
        _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
    }
    else
    {
        if(_ui.arucoPushButton->property("class") != "success" && _ui.arucoPushButton->property("class") != "error") 
        {
            _ui.arucoPushButton->setProperty("class", "normal");
            _ui.arucoPushButton->setEnabled(true);
            _ui.arucoPushButton->style()->unpolish(_ui.arucoPushButton);
            _ui.arucoPushButton->style()->polish(_ui.arucoPushButton);
        }
    }
}

