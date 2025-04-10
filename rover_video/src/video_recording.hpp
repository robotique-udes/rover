#ifndef __VIDEO__RECORDING__
#define __VIDEO__RECORDING__

#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/macros.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#include <atomic>
#include <chrono>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * @brief Each Recording objects handle 1 camera stream \n
 * @brief They must be initialized with startRecording()
 * @attention Cameral URL must be RTSP stream for GStreamer pipeline
 */
class Recording
{
  private:
    static constexpr uint8_t RECORDING_INTERVAL_SHORT_S = 20U;  // in seconds
    static constexpr uint16_t RECORDING_INTERVAL_LONG_S = 900U;  // in seconds (15 minutes)

  public:
    Recording() = delete;

    Recording(std::string videoFolderPath_in,
              std::string filename_in,
              std::string URL_in,
              std::shared_ptr<rclcpp::Logger> logger,
              std::function<void(std::string)> RequestShutdown);

    Recording(Recording&& other);

    Recording& operator=(Recording&& other);

    ~Recording(void);

    bool startRecording(void);
    bool recordFrame(void);

    std::string getURL(void) const;
    std::string getFilename(void) const;
    cv::Mat getFrame(void) const;
    double getFPS(void) const;


  private:
    void recordingThreadFunction(void);

    std::function<void(std::string)> _RequestShutdown;

    std::thread _recordingThread;
    std::atomic<bool> _stopRecording{false};

    // camera variables
    std::string _camURL;
    std::string _pipeline;
    std::string _filename;
    std::string _videoFolderPath;

    std::vector<std::string> _files;

    uint8_t _recordingNumberShort = 1;
    uint8_t _recordingNumberLong = 1;
    time_t _shortTimer;
    time_t _longTimer;

    int _frame_width;
    int _frame_height;
    double _fps;

    // ros logger
    std::shared_ptr<rclcpp::Logger> rLogger;  // allows Recording objects to send logs from ROS nodes

    // cv variables
    cv::VideoCapture _cap;
    cv::VideoWriter _video_writer_short;
    cv::VideoWriter _video_writer_long;
    cv::Mat _frame;
};

#endif