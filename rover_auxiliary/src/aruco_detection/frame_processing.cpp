#include "frame_processing.hpp"

// Init frame processing when accessing camera with ID
FrameProcessing::FrameProcessing(std::string cameraURL_):
    _stream(cameraURL_),
    DICTIONNARY(cv::aruco::getPredefinedDictionary(DICT))
{
    _detectorParams = cv::aruco::DetectorParameters::create();
}

std::optional<cv::Mat> FrameProcessing::processFrame(bool debugMode_)
{
    std::optional<cv::Mat> frame = _stream.getFrame(debugMode_);
    if (!frame)
    {
        RCLCPP_WARN(rclcpp::get_logger("ArucoDetection"), "Error getting frame from stream");
    }
    cv::aruco::detectMarkers(frame.value(), DICTIONNARY, _corners, _ids, _detectorParams);
    _detectedIds.clear();

    for (uint8_t i = 0; i < _ids.rows; ++i)
    {
        _detectedIds.push_back(_ids.at<uint16_t>(i, 0));
    }

    if (debugMode_)
    {
        cv::aruco::drawDetectedMarkers(frame.value(), _corners, _ids);
        return frame;
    }
    return std::nullopt;
}

std::vector<uint16_t> FrameProcessing::getIds()
{
    return _detectedIds;
}

bool FrameProcessing::IdsEmpty(void)
{
    return _detectedIds.empty();
}