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
    bool startRecording();
    bool recordFrame();

    std::string getURL()
    {
        return this->camURL;
    }

    cv::Mat getFrame()
    {
        return this->frame;
    }

    uint8_t getFPS()
    {
        return this->fps;
    }

  private:
    static constexpr uint8_t RECORDING_INTERVAL = 20U;  // in seconds
    void RecordingThreadFunction();
    std::function<void(std::string)> RequestShutdown_;

    std::shared_ptr<std::thread> recordingThread;
    std::atomic<bool> stopRecording{false};

    // camera variables
    std::string camURL;
    std::string pipeline;
    std::string filename;
    std::string videoFolderPath;

    std::vector<std::string> files;

    uint8_t recordingNumber = 1;
    time_t startTime;

    int frame_width;
    int frame_height;
    double fps;

    // ros logger
    std::shared_ptr<rclcpp::Logger> logger_;  // allows Recording objects to send logs from ROS nodes

    // cv variables
    cv::VideoCapture cap;
    cv::VideoWriter video_writer;
    cv::VideoWriter appender;
    cv::Mat frame;

  public:
    Recording(std::string videoFolderPath_in,
              std::string filename_in,
              std::string URL_in,
              std::shared_ptr<rclcpp::Logger> logger,
              std::function<void(std::string)> RequestShutdown):
        RequestShutdown_(RequestShutdown),
        camURL(URL_in),
        filename(filename_in),
        videoFolderPath(videoFolderPath_in),
        logger_(logger)
    {
    }

    Recording(Recording&& other) noexcept:
        // move constructor, used to move the temporary object created by hashmap emplace
        RequestShutdown_(std::move(other.RequestShutdown_)),
        recordingThread(std::move(other.recordingThread)),
        camURL(std::move(other.camURL)),
        pipeline(std::move(other.pipeline)),
        filename(std::move(other.filename)),
        videoFolderPath(std::move(other.videoFolderPath)),
        files(std::move(other.files)),
        recordingNumber(other.recordingNumber),
        startTime(other.startTime),
        frame_width(other.frame_width),
        frame_height(other.frame_height),
        fps(other.fps),
        logger_(std::move(other.logger_)),
        cap(std::move(other.cap)),
        video_writer(std::move(other.video_writer)),
        appender(std::move(other.appender)),
        frame(std::move(other.frame))
    {
        stopRecording.store(other.stopRecording.load());  // cannot move atomic
    }

    Recording& operator=(Recording&& other) noexcept
    {  // move operator just to be safe
        if (this != &other)
        {  // Prevent self-assignment

            // Move resources
            RequestShutdown_ = std::move(other.RequestShutdown_);
            recordingThread = std::move(other.recordingThread);
            stopRecording.store(other.stopRecording.load(std::memory_order_acquire), std::memory_order_release);

            camURL = std::move(other.camURL);
            pipeline = std::move(other.pipeline);
            filename = std::move(other.filename);
            videoFolderPath = std::move(other.videoFolderPath);
            files = std::move(other.files);
            recordingNumber = other.recordingNumber;
            startTime = other.startTime;
            frame_width = other.frame_width;
            frame_height = other.frame_height;
            fps = other.fps;
            logger_ = std::move(other.logger_);

            appender = std::move(other.appender);
            cap = std::move(other.cap);  // Move cv ressources
            video_writer = std::move(other.video_writer);
            frame = std::move(other.frame);
        }
        return *this;
    }

    ~Recording()
    {
        if (cap.isOpened())  // avoid unnecessary logging when creating temporary objects
        {
            stopRecording.store(true);

            if (recordingThread->joinable())
            {
                recordingThread->join();
            }
            // Release resources
            this->cap.release();
            this->video_writer.release();
            this->appender.release();

            RCLCPP_INFO(*logger_, "Recording stopped.");
        }
    }
};

#endif