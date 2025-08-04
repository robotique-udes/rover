#ifndef PANORAMA_WORKER_HPP
#define PANORAMA_WORKER_HPP

#include <QObject>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include <rover_msgs/srv/panorama.hpp>
#include "Global/Workers/QWorker.hpp"

class QPanoramaWorker : public QWorker
{
    Q_OBJECT

  private:
    static constexpr const char* CAMERA_PATH = "/camera";
    static constexpr uint16_t STITCH_TIMEOUT_MS = 2'000U;

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