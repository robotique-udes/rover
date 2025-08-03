#ifndef PANORAMA_WORKER_HPP
#define PANORAMA_WORKER_HPP

#include <QObject>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include <rover_msgs/srv/photo_panoramique.hpp>
#include "Global/Workers/QWorker.hpp"

class QPanoramaWorker : public QWorker
{
    Q_OBJECT

  private:
  public:
    QPanoramaWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QPanoramaWorker();

    void takePanoramaManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::PhotoPanoramique>> client_panoramique_,
                             std::string cameraUrl_,
                             uint16_t playerIndex_,
                             std::string basePath_,
                             uint16_t duration_);

  signals:
    void panoramaStarted(uint16_t duration_, uint16_t playerIndex_);
    void panoramaFinished(bool success_, const std::string& status_, uint16_t playerIndex_);

  private:
    void takePanoramaInternal(std::shared_ptr<rclcpp::Client<rover_msgs::srv::PhotoPanoramique>> client_panoramique_,
                              std::string cameraUrl_,
                              uint16_t playerIndex_,
                              std::string basePath_,
                              uint16_t duration_);
};

#endif  // PANORAMA_WORKER_HPP