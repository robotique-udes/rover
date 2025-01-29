#include "aruco_detection.h"



using namespace ArucoDetectionHelpers;

ImageCapture::ImageCapture(std::string _cameraURL, int _cameraAccessMode) : cameraURL(_cameraURL), cameraAccessMode(_cameraAccessMode) {}
ImageCapture::ImageCapture(int _cameraID, int _cameraAccessMode) : cameraID(_cameraID), cameraAccessMode(_cameraAccessMode) {}
ImageCapture::~ImageCapture() {
    cap.release();
    cv::destroyAllWindows();
}

void ImageCapture::setCameraURL(std::string URL) { cameraURL = URL; }
std::string ImageCapture::getCameraURL() { return cameraURL; }

bool ImageCapture::accessStream() {
    if (cameraAccessMode == CameraAccessMode::URL) {
        cap.open(cameraURL);
    } else {
        cap.open(cameraID);
    }
    return cap.isOpened();
}

bool ImageCapture::manageStream() {
    return cap.isOpened() || accessStream();
}

cv::Mat ImageCapture::getFrame() {
    cv::Mat frame;
    if (!cap.isOpened()) {
        if (!manageStream()) {
            getErrorFrame(frame);
            return frame;
        }
    }
    cap >> frame;
    if (frame.empty()) getErrorFrame(frame);
    return frame;
}

void ImageCapture::getErrorFrame(cv::Mat& frame) {
    frame = cv::Mat::zeros(480, 640, CV_8UC3);
    std::string error_message = "Error: Stream not found!";
    cv::putText(frame, error_message, cv::Point(100, 240), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
}

FrameProcessing::FrameProcessing(std::string _cameraURL, int _cameraAccessMode) : stream(_cameraURL, _cameraAccessMode) {
    dictionary = cv::aruco::getPredefinedDictionary(DICT);
    detectorParams = cv::aruco::DetectorParameters::create();
}

FrameProcessing::FrameProcessing(int _cameraID, int _cameraAccessMode) : stream(_cameraID, _cameraAccessMode) {
    dictionary = cv::aruco::getPredefinedDictionary(DICT);
    detectorParams = cv::aruco::DetectorParameters::create();
}

FrameProcessing::~FrameProcessing() {}

bool FrameProcessing::processFrame() {
    cv::aruco::detectMarkers(stream.getFrame(), dictionary, corners, ids);
    detectedIds.clear();
    for (int i = 0; i < ids.rows; ++i) {
        detectedIds.push_back(ids.at<int>(i, 0));
    }
    return !detectedIds.empty();
}

cv::Mat FrameProcessing::processedFrameToShow() {
    processFrame();
    cv::Mat processedFrame = stream.getFrame();
    cv::aruco::drawDetectedMarkers(processedFrame, corners, ids);
    return processedFrame;
}

std::vector<int> FrameProcessing::getIds() {
    return detectedIds;
}

ArucoDetection::ArucoDetection(std::string _cameraURL, int _cameraAccessMode) : processedFrame(_cameraURL, _cameraAccessMode) {}
ArucoDetection::ArucoDetection(int _cameraID, int _cameraAccessMode) : processedFrame(_cameraID, _cameraAccessMode) {}
ArucoDetection::~ArucoDetection() {}

std::vector<int> ArucoDetection::detect() {
    if (processedFrame.processFrame()) {
        return processedFrame.getIds();
    }
    return {}; // Return empty vector if no IDs detected
}

bool ArucoDetection::detectAndShow() {
    cv::Mat frame = processedFrame.processedFrameToShow();
    cv::imshow("Aruco Detection", frame);
    cv::waitKey(30);
    return true;
}

std::vector<int> ArucoDetection::update() {
    std::vector<int> validatedIds;
    std::vector<int> detectedIds = detect();

    for (const int& id : detectedIds) {
        validationCounts[id]++;
        strike = 0;
    }

    strike++;

    if (strike > 10) {
        validationCounts.clear();
        for (const int& id : detectedIds) { //pour chaque id detecte
            validationCounts[id] = 0;
        }
        strike = 0;
    }

    for (const auto& [id, count] : validationCounts) {
        if (count >= 10) {
            validatedIds.push_back(id);
            validationCounts[id] = 0; // Reset count after validation
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(ArucoDetectionHelpers::DELAY_BETWEEN_CAPTURE_MS));
    return validatedIds;
}



//a fix

ArucoDetectionNode::ArucoDetectionNode() : Node("aurco_detection_node")
{
    _pub_aruco = this->create_publisher<std_msgs::msg::UInt8MultiArray>("detected_arucos", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArucoDetectionNode::update, this));
    detection = new ArucoDetection(0, CameraAccessMode::ID);

}
 
ArucoDetectionNode::~ArucoDetectionNode() {

    delete detection;
}
 
void ArucoDetectionNode::ArucoCallback()
{

    std::vector<int> detectedArucos = detection.update();
 
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = detectedArucos;
 
    _pub_aruco-> publish(msg);
 
    if (!detectedArucos.empty())
    {
        std::string marker_list = "Publishing detected ArUco markers: ";
        for (auto id: detectedArucos)
        {
            marker_list += std::to_string(id) + " ";
        }
        RCLCPP_INFO (this->get_logger(), "%s", marker_list.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No ArUco markers detected to publish");
    }
}
 
int main()
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
