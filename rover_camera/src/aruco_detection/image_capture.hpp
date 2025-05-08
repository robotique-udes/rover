#ifndef IMAGE_CAPTURE_HPP
#define IMAGE_CAPTURE_HPP

#include "rover_lib2/helpers/time.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/ip_pinging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

class ImageCapture
{
    static constexpr uint64_t DELAY_CAMERA_PINNING_RETRY_MS = 5'000UL;
    static constexpr size_t TIMEOUT_CAMERA_PINNING__MS = 500UL;
    static constexpr size_t CAM_NETWORK_PORT = 554U;
    static constexpr const char* PIPELINE = " latency=0 drop-on-latency=true protocols=tcp ! "
                                            "decodebin ! "
                                            "videorate max-rate=5 ! "
                                            "videoconvert ! "
                                            "queue max-size-buffers=1 leaky=downstream ! "
                                            "appsink sync=false";

  public:
    ImageCapture(std::string cameraURL_);
    ~ImageCapture(void);

    bool changeStream(std::string URL_);
    std::optional<cv::Mat> getFrame(bool debugMode_);
    cv::Mat getErrorFrame(void);
    bool initCam(void);
    bool isValid(void) const;
    bool isCameraReachable(const std::string& url_, size_t port_, size_t timeoutMs_);

  private:
    bool _isValid;
    std::string _cameraURL;
    cv::VideoCapture _cap;
    LoopTimer<uint64_t, Time::millis> _timer_cameraPinningRetries;
    bool _firstTryPinningCam = true;

    // The the max rate (fps) must be paired with the detection delay
    std::string _rtspPipeline;
};

#endif
