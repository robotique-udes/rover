#ifndef __FRAME_PROCESSING_HPP__
#define __FRAME_PROCESSING_HPP__

#include "image_capture.hpp"
#include <optional>
#include <vector>

class FrameProcessing
{
  public:
    FrameProcessing(std::string _cameraURL);
    ~FrameProcessing(void);
    bool empty;

    std::optional<cv::Mat> processFrame(bool DEBUG_MODE);
    std::vector<uint16_t> getIds(void);
    bool IdsEmpty(void);

  private:
    static constexpr uint8_t MAX_SAME_FRAME_IDS = 20;
    static constexpr cv::aruco::PREDEFINED_DICTIONARY_NAME DICT = cv::aruco::DICT_4X4_250;
    ImageCapture stream;
    std::vector<uint16_t> detectedIds;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat ids;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
};

#endif