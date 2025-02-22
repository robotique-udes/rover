#ifndef __FRAME_PROCESSING_HPP__
#define __FRAME_PROCESSING_HPP__

#include "image_capture.hpp"
#include <optional>
#include <vector>

class FrameProcessing
{
  static constexpr uint8_t MAX_SAME_FRAME_IDS = 20;
  static constexpr cv::aruco::PREDEFINED_DICTIONARY_NAME DICT = cv::aruco::DICT_4X4_250;

  public:
    FrameProcessing(std::string cameraURL_);
    
    std::optional<cv::Mat> processFrame(bool debugMode_);
    std::vector<uint16_t> getIds(void);
    bool IdsEmpty(void);

    bool empty = true;

  private:

    ImageCapture _stream;

    const cv::Ptr<cv::aruco::Dictionary> DICTIONNARY;

    std::vector<uint16_t> _detectedIds;
    std::vector<std::vector<cv::Point2f>> _corners;
    cv::Mat _ids;
    cv::Ptr<cv::aruco::DetectorParameters> _detectorParams;
};

#endif