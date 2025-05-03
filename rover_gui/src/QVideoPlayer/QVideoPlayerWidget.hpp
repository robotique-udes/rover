#ifndef QVIDEOPLAYERWIDGER_HPP
#define QVIDEOPLAYERWIDGER_HPP

#include "rclcpp/rclcpp.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QTimer>
#include "UI_VideoPlayer.h"
#include "Worker/QPlayerWorker.hpp"

class QVideoPlayerWidget : public QWidget
{
    Q_OBJECT

    static constexpr size_t DELAY_OPENING_CAM_RETRY_MS = 5'000UL;
    static constexpr size_t MAX_DELAY_SERVICE_CALL = 2'000UL;
    static constexpr size_t NBR_IDS_TO_DISPLAY = 5U;
    static constexpr size_t STYLE_RESET_TIME = 2'000UL;

  public:
    QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                       std::string url_,
                       uint16_t tag_,
                       std::shared_ptr<QPlayerWorker> worker_);

    void setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_);

    void startDetection(void);
    void stopDetection(void);
    void handleArucoDetection(void);
    void arucoStillAliveUpdate(bool urlFound_);
    void displayDetectedArucos(std::vector<uint16_t> ids_);

    void handlePlayPauseButton(void);

    /**
     * @brief Set the camera control client for the widget
     *
     * @param client_ a CameraControl client
     */
    void setCameraControlClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> client_);

    std::string getCamURL(void);
    void setCamURL(std::string newCamUrl_);
    void setURLToDefault(void);
    void updateCamURL(void);

  signals:
    void arucoCameraFailure(bool valid_);
    // void statusBarUpdate(std::string message_, size_t duration_);

  private slots:
    void onDetectionHandledSuccessfully(bool success_, uint16_t tag_);
    void onArucoServerInfoFailed(bool success_);
    void onArucoCameraFailed(bool valid_);

    /**
     * @brief takes a screenshot
     *
     */
    void handleScreenshot(void);

    /**
     * @brief Starts and stops the recording
     *
     */
    void handleRecording(void);

    /**
     * @brief Change screenshot button style
     *
     * @param success_ true or false
     * @param status_ if success -> filepath, else -> reason for failure
     * @param tag_ widget number
     */
    void onScreenshotHandledSuccessfully(bool success_, std::string status_, uint16_t tag_);

    /**
     * @brief Change recording button style
     *
     * @param success_ true or false
     * @param status_ if success -> filepath, else ->reason for failure
     * @param tag_ widget number
     */
    void onStartRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t tag_);

    /**
     * @brief Change recording button style
     *
     * @param success_ true or false
     * @param status_ if success -> filepath, else ->reason for failure
     * @param tag_ widget number
     */
    void onStopRecordingHandledSuccessfully(bool success_, std::string status_, uint16_t tag_);    

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::VideoPlayer _ui;

    std::string _camURL = "";
    std::string _defaultCamUrl = "";
    uint16_t _tag;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoManager = nullptr;
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> _client_cameraControlManager = nullptr;

    std::shared_ptr<QPlayerWorker> _playerWorkerThread;
};

#endif  // QVIDEOPLAYERWIDGER_HPP
