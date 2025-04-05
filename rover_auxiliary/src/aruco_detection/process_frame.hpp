#ifndef __FRAME_PROCESSING_HPP__
#define __FRAME_PROCESSING_HPP__

#include "image_capture.hpp"
#include <optional>
#include <vector>

class ProcessFrame
{
    static constexpr uint8_t MAX_SAME_FRAME_IDS = 20;
    static constexpr cv::aruco::PREDEFINED_DICTIONARY_NAME DICT = cv::aruco::DICT_4X4_250;

  public:
    ProcessFrame(std::string cameraURL_);

    std::optional<cv::Mat> updateDetection(bool debugMode_);
    std::vector<uint16_t> getIds(void) const;
    bool IdsEmpty(void) const;
    bool isValid(void) const;

    bool empty = true;

  private:
    const cv::Ptr<cv::aruco::Dictionary> DICTIONNARY;

    ImageCapture _stream;
    std::vector<uint16_t> _detectedIds;
    std::vector<std::vector<cv::Point2f>> _corners;
    cv::Mat _ids;
    cv::Ptr<cv::aruco::DetectorParameters> _detectorParams;
};

#endif