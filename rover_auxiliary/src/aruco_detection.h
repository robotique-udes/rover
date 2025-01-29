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
#include "std_msgs/msg/string.hpp"  // ??

namespace ArucoDetectionHelpers {

    constexpr int MAX_SAME_FRAME_IDS = 20;
    constexpr cv::aruco::PREDEFINED_DICTIONARY_NAME DICT = cv::aruco::DICT_4X4_250;
    constexpr int DELAY_BETWEEN_CAPTURE_MS = 100;

    enum CameraAccessMode { URL = -2, ID = -3 };

    class ImageCapture {
    private:
        std::string cameraURL;
        int cameraID;
        int cameraAccessMode;
        cv::VideoCapture cap;

    public:
        ImageCapture(std::string _cameraURL, int _cameraAccessMode);
        ImageCapture(int _cameraID, int _cameraAccessMode);
        ~ImageCapture();

        void setCameraURL(std::string);
        std::string getCameraURL();

        bool accessStream();
        bool manageStream();
        cv::Mat getFrame();
        void getErrorFrame(cv::Mat&);
    };

    class FrameProcessing {
    private:
        ImageCapture stream;
        std::vector<int> detectedIds;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::Mat ids;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

    public:
        FrameProcessing(std::string _cameraURL, int _cameraAccessMode);
        FrameProcessing(int _cameraID, int _cameraAccessMode);
        ~FrameProcessing();

        bool processFrame();
        cv::Mat processedFrameToShow();
        std::vector<int> getIds();
    };
}

class ArucoDetection {
private:
    ArucoDetectionHelpers::FrameProcessing processedFrame;
    std::vector<int> idsInValidation;
    std::unordered_map<int, int> validationCounts;
    int strike = 0;

public:
    ArucoDetection(int _cameraID, int _cameraAccessMode);
    ArucoDetection(std::string _cameraURL, int _cameraAccessMode);
    
    ~ArucoDetection();

    std::vector<int> detect();
    bool detectAndShow();
    std::vector<int> update();
};





//a fix
 
class ArucoDetectionNode: public rclcpp::Node
{
    public:
        ArucoDetectionNode();
        ~ArucoDetectionNode();
        void ArucoCallback();
 
    private:
        ArucoDetection* detection;
        rclcpp::Publisher<rover_::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        
};

#endif