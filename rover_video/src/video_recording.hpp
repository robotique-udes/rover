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

class Recording
{
  public:
    bool startRecording(void);
    bool recordFrame(void);

    std::string getURL(void) const
    {
        return _camURL;
    }

    cv::Mat getFrame(void) const
    {
        return _frame;
    }

    uint8_t getFPS(void) const
    {
        return _fps;
    }

  private:
    static constexpr uint8_t RECORDING_INTERVAL = 20U;  // in seconds
    void recordingThreadFunction();
    std::function<void(std::string)> _RequestShutdown;

    std::shared_ptr<std::thread> _recordingThread;
    std::atomic<bool> _stopRecording{false};

    // camera variables
    std::string _camURL;
    std::string _pipeline;
    std::string _filename;
    std::string _videoFolderPath;

    std::vector<std::string> _files;

    uint8_t _recordingNumber = 1;
    time_t _startTime;

    int _frame_width;
    int _frame_height;
    double _fps;

    // ros logger
    std::shared_ptr<rclcpp::Logger> rLogger;  // allows Recording objects to send logs from ROS nodes

    // cv variables
    cv::VideoCapture _cap;
    cv::VideoWriter _video_writer;
    cv::VideoWriter _appender;
    cv::Mat _frame;

  public:
    Recording(std::string videoFolderPath_in,
              std::string filename_in,
              std::string URL_in,
              std::shared_ptr<rclcpp::Logger> logger,
              std::function<void(std::string)> RequestShutdown):
        _RequestShutdown(RequestShutdown),
        _camURL(URL_in),
        _filename(filename_in),
        _videoFolderPath(videoFolderPath_in),
        rLogger(logger)
    {
    }

    Recording(Recording&& other) noexcept:
        // move constructor, used to move the temporary object created by hashmap emplace
        _RequestShutdown(std::move(other._RequestShutdown)),
        _recordingThread(std::move(other._recordingThread)),
        _camURL(std::move(other._camURL)),
        _pipeline(std::move(other._pipeline)),
        _filename(std::move(other._filename)),
        _videoFolderPath(std::move(other._videoFolderPath)),
        _files(std::move(other._files)),
        _recordingNumber(other._recordingNumber),
        _startTime(other._startTime),
        _frame_width(other._frame_width),
        _frame_height(other._frame_height),
        _fps(other._fps),
        rLogger(std::move(other.rLogger)),
        _cap(std::move(other._cap)),
        _video_writer(std::move(other._video_writer)),
        _appender(std::move(other._appender)),
        _frame(std::move(other._frame))
    {
        _stopRecording.store(other._stopRecording.load());  // cannot move atomic
    }

    Recording& operator=(Recording&& other) noexcept
    {  // move operator just to be safe
        if (this != &other)
        {  // Prevent self-assignment

            // Move resources
            _RequestShutdown = std::move(other._RequestShutdown);
            _recordingThread = std::move(other._recordingThread);
            _stopRecording.store(other._stopRecording.load(std::memory_order_acquire), std::memory_order_release);

            _camURL = std::move(other._camURL);
            _pipeline = std::move(other._pipeline);
            _filename = std::move(other._filename);
            _videoFolderPath = std::move(other._videoFolderPath);
            _files = std::move(other._files);
            _recordingNumber = other._recordingNumber;
            _startTime = other._startTime;
            _frame_width = other._frame_width;
            _frame_height = other._frame_height;
            _fps = other._fps;
            rLogger = std::move(other.rLogger);

            _appender = std::move(other._appender);
            _cap = std::move(other._cap);  // Move cv ressources
            _video_writer = std::move(other._video_writer);
            _frame = std::move(other._frame);
        }
        return *this;
    }

    ~Recording()
    {
        if (_cap.isOpened())  // avoid unnecessary logging when creating temporary objects
        {
            _stopRecording.store(true);

            if (_recordingThread->joinable())
            {
                _recordingThread->join();
            }
            // Release resources
            _cap.release();
            _video_writer.release();
            _appender.release();

            RCLCPP_INFO(*rLogger, "Recording stopped.");
        }
    }
};

#endif