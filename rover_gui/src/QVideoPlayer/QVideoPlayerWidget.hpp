#ifndef __QVIDEOPLAYERWIDGER_HPP__
#define __QVIDEOPLAYERWIDGER_HPP__

#include "rclcpp/rclcpp.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include "UI_VideoPlayer.h"
#include "Worker/QPlayerWorker.hpp"

class QVideoPlayerWidget : public QWidget
{
    Q_OBJECT

    static constexpr uint64_t MAX_DELAY_SERVICE_CALL = 2000UL;
    static constexpr uint16_t NBR_IDS_TO_DISPLAY = 5U;

  public:
    QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_,
                       QWidget* parent_,
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

    std::string getCamURL(void);
    void setCamURL(std::string newCamUrl_);
    void setURLToDefault(void);
    void updateCamURL(void);

  signals:
    void arucoCameraFailure(bool valid_);

  private slots:
    void onDetectionHandledSuccessfully(bool success_, uint16_t tag_);
    void onArucoServerInfoFailed(bool success_);
    void onArucoCameraFailed(bool valid_);

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::VideoPlayer _ui;

    std::string _camURL = "";
    std::string _defaultCamUrl = "";
    uint16_t _tag;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoManager = nullptr;

    std::shared_ptr<QPlayerWorker> _playerWorkerThread;
};

#endif  // __QVIDEOPLAYERWIDGER_HPP___
