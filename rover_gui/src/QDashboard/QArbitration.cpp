#include "QArbitration.hpp"

void QArbitration::sendRequest(eControllerType controllerType_, eDemuxDestination demuxDestination_)
{
    if (!_clientJoy->wait_for_service(std::chrono::seconds(1))) {
        setControlButtonsEnabled(false);

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setWindowTitle("Erreur de connexion");
        msgBox.setText("Le service de téléopération (/demux_control) est inaccessible.");
        msgBox.exec();

        _ui.checkBox_2->setChecked(true);
        _ui.checkBox_8->setChecked(true);

        
        if (!_reconnectTimer) {
            _reconnectTimer = new QTimer(this);
            connect(_reconnectTimer, &QTimer::timeout, this, [this]() {
                if (_clientJoy->service_is_ready()) {
                    setControlButtonsEnabled(true);

                    QMessageBox::information(this, "Service rétabli", "Le service /demux_control est à nouveau disponible.");

                    _reconnectTimer->stop();
                    _reconnectTimer->deleteLater();
                    _reconnectTimer = nullptr;
                }
            });
            _reconnectTimer->start(1000);  
        }

        return;
    }
    
    setControlButtonsEnabled(true);

    auto request = std::make_shared<rover_msgs::srv::JoyDemuxSetState::Request>();

    request->controller_type = controllerType_;
    request->destination = demuxDestination_;

    if (controllerType_== QArbitration::eControllerType::main){
        request->force = true;
    }

    auto result = _clientJoy->async_send_request(request);
}

void QArbitration::setControlButtonsEnabled(bool enabled)
{
    //MAIN
    _ui.checkBox->setEnabled(enabled);     // Drive Train
    _ui.checkBox_3->setEnabled(enabled);   // Arm
    _ui.checkBox_4->setEnabled(enabled);   // Antenna

    // SECONDARY 
    _ui.checkBox_5->setEnabled(enabled);   // Drive Train
    _ui.checkBox_7->setEnabled(enabled);   // Arm
    _ui.checkBox_6->setEnabled(enabled);   // Antenna
}


void QArbitration::joyDemuxCallback(const rover_msgs::msg::JoyDemuxStatus::SharedPtr rosMsg_)
{

    switch(rosMsg_->controller_main_topic) {
        case 0: _ui.checkBox->setChecked(true); break;
        case 1: _ui.checkBox_3->setChecked(true);break;
        case 2: _ui.checkBox_4->setChecked(true);break;
        case 3: _ui.checkBox_2->setChecked(true); break;
        default: break;
    }

    switch(rosMsg_->controller_secondary_topic){
        case 0: _ui.checkBox_5->setChecked(true); break;
        case 1: _ui.checkBox_7->setChecked(true); break;
        case 2: _ui.checkBox_6->setChecked(true); break;
        case 3: _ui.checkBox_8->setChecked(true); break;
        default: break;
    }
}

