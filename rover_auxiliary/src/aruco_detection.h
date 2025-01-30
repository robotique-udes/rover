#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/aruco.hpp"


namespace ArucoDetectionHelpers {

    constexpr uint8_t MAX_SAME_FRAME_IDS = 20;
    constexpr cv::aruco::PREDEFINED_DICTIONARY_NAME DICT = cv::aruco::DICT_4X4_250;
    constexpr uint8_t DELAY_BETWEEN_CAPTURE_MS = 100;

    enum CameraAccessMode : uint8_t { URL = 0, ID = 1 };

    class ImageCapture {
    private:
        std::string cameraURL;
        uint8_t cameraID;
        uint8_t cameraAccessMode;
        cv::VideoCapture cap;

    public:
        ImageCapture(std::string _cameraURL, uint8_t _cameraAccessMode);
        ImageCapture(uint8_t _cameraID, uint8_t _cameraAccessMode);
        ~ImageCapture();

        void setCameraURL(std::string);
        std::string getCameraURL();

        bool accessStream();
        bool manageStream();
        cv::Mat getFrame();
       // void getErrorFrame(cv::Mat&);  Use for debugging
    };

    class FrameProcessing {
    private:
        ImageCapture stream;
        std::vector<uint16_t> detectedIds;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::Mat ids;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

    public:
        FrameProcessing(std::string _cameraURL, uint8_t _cameraAccessMode);
        FrameProcessing(uint8_t _cameraID, uint8_t _cameraAccessMode);
        ~FrameProcessing();

        bool processFrame();
        //cv::Mat processedFrameToShow();  Use for debugging
        std::vector<uint16_t> getIds();
    };
}

class ArucoDetection {
private:
    ArucoDetectionHelpers::FrameProcessing processedFrame;
    std::vector<uint16_t> idsInValidation;
    std::unordered_map<uint16_t, uint16_t> validationCounts;
    uint8_t strike = 0;

public:
    ArucoDetection(uint8_t _cameraID, uint8_t _cameraAccessMode);
    ArucoDetection(std::string _cameraURL, uint8_t _cameraAccessMode);
    ~ArucoDetection();
    std::vector<uint16_t> detect();
    // bool detectAndShow();   Use for debugging
    std::vector<uint16_t> update();
};


class ArucoDetectionNode: public rclcpp::Node
{
    public:
        ArucoDetectionNode();
        ~ArucoDetectionNode();
 
    private:
        void ArucoCallback();
        rclcpp::Publisher<rover_msgs::msg::Aruco>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<ArucoDetection> detection_;
};

#endif