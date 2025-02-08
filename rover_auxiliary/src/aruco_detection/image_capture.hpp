#ifndef __IMAGE_CAPTURE_HPP__
#define __IMAGE_CAPTURE_HPP__

#include "rclcpp/rclcpp.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

class ImageCapture
{
  public:
    ImageCapture(std::string _cameraURL = "v4l2:///dev/video0");
    ~ImageCapture(void);

    bool manageStream(std::string URL);
    cv::Mat getFrame(bool DEBUG_MODE);
    void getErrorFrame(cv::Mat&);

    bool initCam();

  private:
    std::string cameraURL;
    cv::VideoCapture cap;
};

#endif