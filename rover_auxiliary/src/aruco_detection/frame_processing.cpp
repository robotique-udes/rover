#include "frame_processing.hpp"

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