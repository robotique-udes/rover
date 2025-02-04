#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/aruco.hpp"
#include "rovus_lib/moving_average.hpp"
#include "rovus_lib/timer.hpp"
#include <chrono>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

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

class FrameProcessing
{
  public:
    FrameProcessing(std::string _cameraURL);
    ~FrameProcessing(void);
    bool empty;

    std::optional<cv::Mat> processFrame(bool DEBUG_MODE);
    std::vector<uint16_t> getIds(void);
    bool IdsEmpty(void);

  private:
    static constexpr uint8_t MAX_SAME_FRAME_IDS = 20;
    static constexpr cv::aruco::PREDEFINED_DICTIONARY_NAME DICT = cv::aruco::DICT_4X4_250;
    ImageCapture stream;
    std::vector<uint16_t> detectedIds;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat ids;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
};

class ArucoDetection
{
  public:
    ArucoDetection(std::string _cameraURL);
    ~ArucoDetection(void);
    std::vector<uint16_t> detect(bool DEBUG_MODE);
    std::vector<uint16_t> update(bool DEBUG_MODE);

  private:
#define COEFF_NB 10
    static constexpr float VALIDATION_THRESHOLD = 0.75;
    static constexpr uint8_t DELAY_BETWEEN_CAPTURE_MS = 100;
    FrameProcessing processedFrame;
    std::unordered_map<uint16_t, MovingAverage<uint16_t, COEFF_NB>> validation;
};

class ArucoDetectionNode : public rclcpp::Node
{
  public:
    ArucoDetectionNode(void);
    ~ArucoDetectionNode(void);

  private:
    static constexpr bool DEBUG_MODE = true;
    void ArucoCallback(void);
    rclcpp::Publisher<rover_msgs::msg::Aruco>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<ArucoDetection> detection_;
};

#endif