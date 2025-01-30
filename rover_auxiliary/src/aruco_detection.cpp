#include "aruco_detection.h"


int main(int  argc, char **argv)
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectionNode>());
    rclcpp::shutdown();
    return 0;
}



using namespace ArucoDetectionHelpers;

ImageCapture::ImageCapture(std::string _cameraURL, uint8_t  _cameraAccessMode) : cameraURL(_cameraURL), cameraAccessMode(_cameraAccessMode) {} 

ImageCapture::ImageCapture(uint8_t  _cameraID, uint8_t  _cameraAccessMode) : cameraID(_cameraID), cameraAccessMode(_cameraAccessMode) {}

ImageCapture::~ImageCapture() {
    cap.release();
    //cv::destroyAllWindows(); Use when debugging
}

void ImageCapture::setCameraURL(std::string URL) { cameraURL = URL; }

std::string ImageCapture::getCameraURL() { return cameraURL; }

bool ImageCapture::accessStream() {

    if (cameraAccessMode == CameraAccessMode::URL) {
        cap.open(cameraURL);
    } 
    
    else {
        cap.open(cameraID);
    }

    return cap.isOpened(); // True if camera is opened
}

bool ImageCapture::manageStream() {
    return accessStream();  // Access camera and return result
}

cv::Mat ImageCapture::getFrame() {
    cv::Mat frame;
    
    if (!cap.isOpened()) {
        if (!manageStream()) {    // Open camera only if it's off
            //getErrorFrame(frame); Use when debugging
            return frame;
        }
    }

    cap >> frame;   // Store frame in matrix  (openCV syntax)
    return frame;
}

/* Use when debugging
void ImageCapture::getErrorFrame(cv::Mat& frame) {
    frame = cv::Mat::zeros(480, 640, CV_8UC3);
    std::string error_message = "Error: Stream not found!";
    cv::putText(frame, error_message, cv::Point(100, 240), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
}*/

// Init frame processing when accessing camera with ID
FrameProcessing::FrameProcessing(std::string _cameraURL, uint8_t  _cameraAccessMode) : stream(_cameraURL, _cameraAccessMode) {
    dictionary = cv::aruco::getPredefinedDictionary(DICT);
    detectorParams = cv::aruco::DetectorParameters::create();
}

// Init frame processing when accessing camera with URL
FrameProcessing::FrameProcessing(uint8_t  _cameraID, uint8_t  _cameraAccessMode) : stream(_cameraID, _cameraAccessMode) {
    dictionary = cv::aruco::getPredefinedDictionary(DICT);
    detectorParams = cv::aruco::DetectorParameters::create();
}

FrameProcessing::~FrameProcessing() {}

bool FrameProcessing::processFrame() {
    cv::aruco::detectMarkers(stream.getFrame(), dictionary, corners, ids);
    detectedIds.clear();

    for (uint8_t  i = 0; i < ids.rows; ++i) {
        detectedIds.push_back(ids.at<uint16_t>(i, 0));
    }

    return !detectedIds.empty(); // Return false if no id detected
}

/* Use when debugging
cv::Mat FrameProcessing::processedFrameToShow() {
    processFrame();
    cv::Mat processedFrame = stream.getFrame();
    cv::aruco::drawDetectedMarkers(processedFrame, corners, ids);
    return processedFrame;
}*/

std::vector<uint16_t > FrameProcessing::getIds() {
    return detectedIds;
}

ArucoDetection::ArucoDetection(std::string _cameraURL, uint8_t  _cameraAccessMode) : processedFrame(_cameraURL, _cameraAccessMode) {}
ArucoDetection::ArucoDetection(uint8_t  _cameraID, uint8_t  _cameraAccessMode) : processedFrame(_cameraID, _cameraAccessMode) {}
ArucoDetection::~ArucoDetection() {}

std::vector<uint16_t > ArucoDetection::detect() {
    if (processedFrame.processFrame()) {    // If ids were detected
        return processedFrame.getIds();
    }

    return {}; // Return empty vector if no ids detected
}


/* Use when debugging
bool ArucoDetection::detectAndShow() {
    cv::Mat frame = processedFrame.processedFrameToShow();
    cv::imshow("Aruco Detection", frame);
    cv::waitKey(30);
    return true;
}*/ 


/*
Look for marker in a new frame every 100 ms
Id has to be detected 10 times to be validated (not necessarily consective)
If Id is not found in the next frame, it has 10 frame to be found again (strike), else the count is reset 
*/
std::vector<uint16_t > ArucoDetection::update() {
    
    std::vector<uint16_t > validatedIds;
    std::vector<uint16_t > detectedIds = detect();

    for (const uint16_t & id : detectedIds) {
        validationCounts[id]++;
        strike = 0;
    }

    strike++;

    if (strike > 10) {
        validationCounts.clear();
        for (const uint16_t & id : detectedIds) {
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


ArucoDetectionNode::ArucoDetectionNode() : Node("aurco_detection_node")
{
    publisher_ = this->create_publisher<rover_msgs::msg::Aruco>("detected_arucos", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArucoDetectionNode::ArucoCallback, this));
    detection_ = std::make_unique<ArucoDetection>(0, CameraAccessMode::ID);
}
    
ArucoDetectionNode::~ArucoDetectionNode() {}
 
void ArucoDetectionNode::ArucoCallback()
{
    std::vector<uint16_t> detectedArucos = detection_->update();

       rover_msgs::msg::Aruco msg;

        msg.valid = !detectedArucos.empty();

        publisher_->publish(msg);

        if (!detectedArucos.empty())
        {
            std::string marker_list = "Publishing detected Aruco markers: ";
            for (auto id : detectedArucos)
            {
                marker_list += std::to_string(id) + " ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", marker_list.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No Aruco markers detected to publish");
        }
}
    

 
