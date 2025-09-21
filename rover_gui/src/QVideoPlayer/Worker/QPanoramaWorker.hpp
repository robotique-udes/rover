#ifndef PANORAMA_WORKER_HPP
#define PANORAMA_WORKER_HPP

#include "rclcpp/rclcpp.hpp"
#include "Global/Workers/QWorker.hpp"

#include <QObject>
#include <QString>
#include <rover_msgs/srv/panorama.hpp>

class QPanoramaWorker : public QWorker
{
    Q_OBJECT

  private:
    static constexpr const char* CAMERA_PATH = "/camera";
    static constexpr std::chrono::milliseconds SERVICE_TIMEOUT_MS = std::chrono::milliseconds(
        4'000U);  // Additional time to account for initializing camera position, stitching the frames and correction warp

  public:
    QPanoramaWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QPanoramaWorker();

    void takePanoramaManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::Panorama>> client_panoramique_,
                             const std::string& cameraUrl_,
                             uint16_t playerIndex_,
                             const std::string& basePath_,
                             uint16_t duration_);

  signals:
    void panoramaStarted(uint16_t duration_, uint16_t playerIndex_);
    void panoramaFinished(bool success_, const std::string& status_, uint16_t playerIndex_);

  private:
    void takePanoramaInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::Panorama>> client_panoramique_,
                              const std::string& cameraUrl_,
                              uint16_t playerIndex_,
                              const std::string& basePath_,
                              uint16_t duration_);
};

#endif  // PANORAMA_WORKER_HPP