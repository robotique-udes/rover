#include "aruco_detection.h"

ImageCapture::ImageCapture(std::string _cameraURL): cameraURL(_cameraURL)
{
    initCam();
}

ImageCapture::~ImageCapture(void)
{
    cap.release();
    cv::destroyAllWindows();
}

bool ImageCapture::initCam()
{
    if (!cap.isOpened())
    {
        cap.open(cameraURL);
    }

    else
        return true;

    if (!cap.isOpened())
    {
        std::string message = "Could not open streaming device";
        RCLCPP_WARN(rclcpp::get_logger("ArucoDetection"), message.c_str());
        return false;
    }
    return true;
}

bool ImageCapture::manageStream(std::string URL)
{
    if (URL != cameraURL)
    {
        cap.release();
        cameraURL = URL;

        if (!initCam())
        {
            std::string message = "Could not change streaming device";
            RCLCPP_WARN(rclcpp::get_logger("ArucoDetection"), message.c_str());
            return false;
        }
        return true;
    }
    else
    {
        return initCam();
    }
}

cv::Mat ImageCapture::getFrame(bool DEBUG_MODE)
{
    cv::Mat frame;

    if (!cap.isOpened())
    {
        if (!manageStream(cameraURL))
        {  // Open camera only if it's off
            if (DEBUG_MODE)
            {
                getErrorFrame(frame);
            }

            return frame;
        }
    }

    cap >> frame;  // Store frame in matrix  (openCV syntax)
    return frame;
}

void ImageCapture::getErrorFrame(cv::Mat& frame)
{
    frame = cv::Mat::zeros(480, 640, CV_8UC3);
    std::string error_message = "Error: Stream not found!";
    cv::putText(frame, error_message, cv::Point(100, 240), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
}

// Init frame processing when accessing camera with ID
FrameProcessing::FrameProcessing(std::string _cameraURL): stream(_cameraURL)
{
    dictionary = cv::aruco::getPredefinedDictionary(DICT);
    detectorParams = cv::aruco::DetectorParameters::create();
}

FrameProcessing::~FrameProcessing(void) {}

std::optional<cv::Mat> FrameProcessing::processFrame(bool DEBUG_MODE)
{
    cv::aruco::detectMarkers(stream.getFrame(DEBUG_MODE), dictionary, corners, ids);
    detectedIds.clear();

    for (uint8_t i = 0; i < ids.rows; ++i)
    {
        detectedIds.push_back(ids.at<uint16_t>(i, 0));
    }

    if (DEBUG_MODE)
    {
        cv::Mat processedFrame = stream.getFrame(DEBUG_MODE);
        cv::aruco::drawDetectedMarkers(processedFrame, corners, ids);
        return processedFrame;
    }
    empty = detectedIds.empty();
    return std::nullopt;  // tjr 0!!
}

std::vector<uint16_t> FrameProcessing::getIds()
{
    return detectedIds;
}

bool FrameProcessing::IdsEmpty(void)
{
    return empty;
}

ArucoDetection::ArucoDetection(std::string _cameraURL): processedFrame(_cameraURL) {}
ArucoDetection::~ArucoDetection() {}

std::vector<uint16_t> ArucoDetection::detect(bool DEBUG_MODE)
{
    processedFrame.processFrame(DEBUG_MODE);
    if (!processedFrame.IdsEmpty())
    {  // If ids were detected
        if (DEBUG_MODE)
        {
            cv::Mat frame = processedFrame.processFrame(DEBUG_MODE).value_or(cv::Mat());
            cv::imshow("Aruco Detection", frame);
            cv::waitKey(30);
        }

        return processedFrame.getIds();
    }

    return {};  // Return empty vector if no ids detected
}

/*
Look for marker in a new frame every 100 ms
Id has to be detected 10 times to be validated (not necessarily consective)
If Id is not found in the next frame, it has 10 frame to be found again (strike), else the count is reset
*/
std::vector<uint16_t> ArucoDetection::update(bool DEBUG_MODE)
{
    std::vector<uint16_t> validatedIds;
    std::vector<uint16_t> detectedIds = detect(DEBUG_MODE);

    for (auto it = validation.begin(); it != validation.end();)
    {
        uint16_t id = it->first;

        auto found = std::find(detectedIds.begin(), detectedIds.end(), id);

        if (found != detectedIds.end())
        {
            it->second.addValue(static_cast<uint16_t>(1));
            detectedIds.erase(found);

            if (it->second.getAverage() > VALIDATION_THRESHOLD)
            {
                validatedIds.push_back(id);
            }

            ++it;
        }
        else 
        {
            it->second.addValue(static_cast<uint16_t>(0));

            if (it->second.getAverage() == 0)
            {
                it = validation.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    for (const auto& id : detectedIds)
    {
        validation.emplace(id, MovingAverage<uint16_t, COEFF_NB>(0));
    }

    RoverLib::Timer<unsigned long, RoverLib::millis> timer(DELAY_BETWEEN_CAPTURE_MS);
    while (!timer.isDone())
    {
    }
    return validatedIds;
}

ArucoDetectionNode::ArucoDetectionNode(void): Node("aurco_detection_node")
{
    publisher_ = this->create_publisher<rover_msgs::msg::Aruco>("detected_arucos", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArucoDetectionNode::ArucoCallback, this));
    detection_ = std::make_unique<ArucoDetection>("v4l2:///dev/video0");
}

ArucoDetectionNode::~ArucoDetectionNode(void) {}

void ArucoDetectionNode::ArucoCallback(void)
{
    std::vector<uint16_t> detectedArucos = detection_->update(DEBUG_MODE);

    rover_msgs::msg::Aruco msg;

    for (auto id : detectedArucos)
    {
        msg.id.push_back(static_cast<uint8_t>(id));  // Ensure ID is cast to uint8
    }

    publisher_->publish(msg);

    if (DEBUG_MODE)
    {
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
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
