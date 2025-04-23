#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "QLoggingMacros.hpp"
#include <QMessageBox>
#include <QStyle>

void RtspPlayerWidget::onArucoButtonClicked(void)
{
    if (!this->_arucoDetectionClient)
    {
        LOG_WARNING_TARGET("RtspPlayer", "Cannot activate Aruco detection - no detection manager set", this->_widgetId.toUtf8().constData());
        QMessageBox::warning(this, "Aruco Detection", "Aruco detection manager not set.");
        this->_arucoButton->setChecked(false);
        return;
    }
    
    // Create player worker thread if it doesn't exist
    if (!this->_playerWorkerThread)
    {
        this->_playerWorkerThread = std::make_shared<QPlayerWorker>(true);
        connect(this->_playerWorkerThread.get(), &QPlayerWorker::detectionHandledSuccessfully,
                this, &RtspPlayerWidget::onDetectionHandledSuccessfully);
        connect(this->_playerWorkerThread.get(), &QPlayerWorker::arucoServerInfoFailed,
                this, &RtspPlayerWidget::onArucoServerInfoFailed);
        LOG_INFO_TARGET("RtspPlayer", "Created player worker thread", this->_widgetId.toUtf8().constData());
    }

    // Toggle detection based on button state
    if (this->_arucoButton->isChecked())
    {
        this->startArucoDetection();
    }
    else
    {
        this->stopArucoDetection();
    }
}

void RtspPlayerWidget::startArucoDetection(void)
{
    if (!this->_playerWorkerThread || !this->_arucoDetectionClient)
    {
        LOG_WARNING_TARGET("RtspPlayer", "Cannot start Aruco detection - dependencies not set", this->_widgetId.toUtf8().constData());
        return;
    }
    
    std::string url = this->_ui.rtspUrlInput->text().toStdString();
    LOG_INFO_TARGET("RtspPlayer", QString("Starting Aruco detection on %1").arg(this->_ui.rtspUrlInput->text()), this->_widgetId.toUtf8().constData());
    
    // Request detection via worker thread
    this->_playerWorkerThread->manageDetection(this->_arucoDetectionClient, url, this->_tag, true);
    
    // Update UI state
    if (!this->_arucoButton->isChecked())
    {
        this->_arucoButton->setChecked(true);
    }
    
    // Style the button to indicate active state
    this->setArucoButtonStyle("success", "#5cb85c");
}

void RtspPlayerWidget::stopArucoDetection(void)
{
    if (!this->_playerWorkerThread || !this->_arucoDetectionClient)
    {
        LOG_WARNING_TARGET("RtspPlayer", "Cannot stop Aruco detection - dependencies not set", this->_widgetId.toUtf8().constData());
        return;
    }
    
    std::string url = this->_ui.rtspUrlInput->text().toStdString();
    LOG_INFO_TARGET("RtspPlayer", QString("Stopping Aruco detection on %1").arg(this->_ui.rtspUrlInput->text()), this->_widgetId.toUtf8().constData());
    
    // Request stop detection via worker thread
    this->_playerWorkerThread->manageDetection(this->_arucoDetectionClient, url, this->_tag, false);
    
    // Update UI state
    if (this->_arucoButton->isChecked())
    {
        this->_arucoButton->setChecked(false);
    }
    
    // Reset button style
    this->setArucoButtonStyle("normal");
}

void RtspPlayerWidget::setArucoButtonStyle(const QString& styleClass_, const QString& bgColor_)
{
    this->_arucoButton->setProperty("class", styleClass_);
    
    if (bgColor_.isEmpty())
    {
        this->_arucoButton->setStyleSheet("");
    }
    else
    {
        this->_arucoButton->setStyleSheet(QString("background-color: %1; color: white;").arg(bgColor_));
    }
    
    this->_arucoButton->style()->unpolish(this->_arucoButton);
    this->_arucoButton->style()->polish(this->_arucoButton);
}

void RtspPlayerWidget::displayDetectedArucos(const std::vector<uint16_t>& ids_)
{
    // Limit the number of IDs displayed
    std::vector<uint16_t> idsToShow = ids_;
    if (idsToShow.size() > NBR_IDS_TO_DISPLAY)
    {
        idsToShow.resize(NBR_IDS_TO_DISPLAY);
    }
    
    // Update the text display
    this->_arucoIdsTextBox->setText("Ids: ");
    for (const auto& id : idsToShow)
    {
        this->_arucoIdsTextBox->setText(this->_arucoIdsTextBox->text() + "  " + QString::number(id));
    }
    
    // Set the hasIds property based on whether IDs were detected
    if (!idsToShow.empty()) {
        this->_arucoIdsTextBox->setProperty("hasIds", true);
    } else {
        this->_arucoIdsTextBox->setProperty("hasIds", false);
    }
    this->_arucoIdsTextBox->style()->unpolish(this->_arucoIdsTextBox);
    this->_arucoIdsTextBox->style()->polish(this->_arucoIdsTextBox);
}

void RtspPlayerWidget::setArucoDetectionManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_)
{
    if (client_ != nullptr)
    {
        this->_arucoDetectionClient = client_;
        
        // Create player worker thread if needed
        if (!this->_playerWorkerThread)
        {
            this->_playerWorkerThread = std::make_shared<QPlayerWorker>(true);
            connect(this->_playerWorkerThread.get(), &QPlayerWorker::detectionHandledSuccessfully,
                    this, &RtspPlayerWidget::onDetectionHandledSuccessfully);
            connect(this->_playerWorkerThread.get(), &QPlayerWorker::arucoServerInfoFailed,
                    this, &RtspPlayerWidget::onArucoServerInfoFailed);
            LOG_INFO_TARGET("RtspPlayer", "Created player worker thread", this->_widgetId.toUtf8().constData());
        }
        
        LOG_INFO_TARGET("RtspPlayer", "Aruco detection manager set", this->_widgetId.toUtf8().constData());
    }
    else
    {
        LOG_WARNING_TARGET("RtspPlayer", "Error, couldn't access aruco detection manager client", this->_widgetId.toUtf8().constData());
        this->setArucoButtonStyle("error", "#d9534f");
    }
}

void RtspPlayerWidget::arucoStillAliveUpdate(bool urlFound_)
{
    if (!urlFound_ && this->_arucoButton->isChecked())
    {
        LOG_WARNING_TARGET("RtspPlayer", QString("Error, aruco detection on %1 was not found").arg(this->_ui.rtspUrlInput->text()), this->_widgetId.toUtf8().constData());
        this->setArucoButtonStyle("normal");
    }
}

void RtspPlayerWidget::onDetectionHandledSuccessfully(bool success_, uint16_t tag_)
{
    if (!success_ && this->_tag == tag_)
    {
        LOG_WARNING_TARGET("RtspPlayer", QString("Error, request made on %1 regarding aruco detection failed").arg(this->_ui.rtspUrlInput->text()), this->_widgetId.toUtf8().constData());
        this->setArucoButtonStyle("error", "#d9534f");
    }
}

void RtspPlayerWidget::onArucoServerInfoFailed(bool success_)
{
    if (!success_)
    {
        LOG_WARNING_TARGET("RtspPlayer", "Error, info request to aruco detection manager client failed", this->_widgetId.toUtf8().constData());
        this->_arucoButton->setEnabled(false);
    }
    else
    {
        if (this->_arucoButton->property("class") != "success" && this->_arucoButton->property("class") != "error")
        {
            this->setArucoButtonStyle("normal");
            this->_arucoButton->setEnabled(true);
        }
    }
}

void RtspPlayerWidget::onArucoCameraFailed(bool valid_)
{
    // Ensure button is checked and enabled for consistent UI state
    if (!this->_arucoButton->isChecked())
    {
        this->_arucoButton->setChecked(true);
    }
    
    if (!this->_arucoButton->isEnabled())
    {
        this->_arucoButton->setEnabled(true);
    }
    
    if (!valid_)
    {
        LOG_WARNING_TARGET("RtspPlayer", QString("Error, camera at %1 is not accessible").arg(this->_ui.rtspUrlInput->text()), this->_widgetId.toUtf8().constData());
        this->setArucoButtonStyle("error", "#d9534f");
    }
    else
    {
        this->setArucoButtonStyle("success", "#5cb85c");
    }
}