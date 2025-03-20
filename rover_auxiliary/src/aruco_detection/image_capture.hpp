#ifndef __IMAGE_CAPTURE_HPP__
#define __IMAGE_CAPTURE_HPP__

#include "rclcpp/rclcpp.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

class ImageCapture
{
  public:
    ImageCapture(std::string cameraURL_);
    ~ImageCapture(void);

    bool changeStream(std::string URL_);
    std::optional<cv::Mat> getFrame(bool debugMode_);
    cv::Mat getErrorFrame(void);
    bool initCam(void);

  private:
    std::string _pipeline;
    std::string _cameraURL;
    cv::VideoCapture _cap;

    // The the max rate (fps) must be paired with the delay between capture
    const std::string _PIPELINE
        = " latency=0 drop=true ! decodebin ! videorate max-rate=5 ! videoconvert ! queue max-size-buffers=1 ! appsink";
    std::string _rtspPipeline;
};

#endif
