#include "frame_processing.hpp"

// Init frame processing when accessing camera with ID
FrameProcessing::FrameProcessing(std::string cameraURL_): _stream(cameraURL_)
{
    _dictionary = cv::aruco::getPredefinedDictionary(DICT);
    _detectorParams = cv::aruco::DetectorParameters::create();
}

FrameProcessing::~FrameProcessing(void) {}

std::optional<cv::Mat> FrameProcessing::processFrame(bool DEBUG_MODE)
{
    std::optional<cv::Mat> frame = _stream.getFrame(DEBUG_MODE);
    if (!frame)
    {
        RCLCPP_WARN(rclcpp::get_logger("ArucoDetection"), "Error getting frame from stream");
    }
    cv::aruco::detectMarkers(frame.value(), _dictionary, _corners, _ids);
    _detectedIds.clear();

    for (uint8_t i = 0; i < _ids.rows; ++i)
    {
        _detectedIds.push_back(_ids.at<uint16_t>(i, 0));
    }

    if (DEBUG_MODE)
    {
        cv::aruco::drawDetectedMarkers(frame.value(), _corners, _ids);
        return frame;
    }
    empty = _detectedIds.empty();
    return std::nullopt;
}

std::vector<uint16_t> FrameProcessing::getIds()
{
    return _detectedIds;
}

bool FrameProcessing::IdsEmpty(void)
{
    return empty;
}
