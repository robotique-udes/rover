#ifndef __IMAGE_CAPTURE_HPP__
#define __IMAGE_CAPTURE_HPP__

#include "rclcpp/rclcpp.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

class ImageCapture
{
  public:
    ImageCapture(std::string cameraURL_ = "v4l2:///dev/video0");
    ~ImageCapture(void);

    bool manageStream(std::string URL_);
    std::optional<cv::Mat> getFrame(bool DEBUG_MODE);
    void getErrorFrame(cv::Mat& frame_);

    bool initCam();

  private:
    std::string _cameraURL;
    cv::VideoCapture _cap;
};

#endif
