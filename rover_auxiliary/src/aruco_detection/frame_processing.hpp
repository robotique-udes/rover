#ifndef __FRAME_PROCESSING_HPP__
#define __FRAME_PROCESSING_HPP__

#include "image_capture.hpp"
#include <optional>
#include <vector>

class FrameProcessing
{
  public:
    FrameProcessing(std::string cameraURL_);
    ~FrameProcessing(void);
    bool empty;

    std::optional<cv::Mat> processFrame(bool DEBUG_MODE);
    std::vector<uint16_t> getIds(void);
    bool IdsEmpty(void);

  private:
    static constexpr uint8_t MAX_SAME_FRAME_IDS = 20;
    static constexpr cv::aruco::PREDEFINED_DICTIONARY_NAME DICT = cv::aruco::DICT_4X4_250;
    ImageCapture _stream;
    std::vector<uint16_t> _detectedIds;
    std::vector<std::vector<cv::Point2f>> _corners;
    cv::Mat _ids;
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> _detectorParams;
};

#endif
