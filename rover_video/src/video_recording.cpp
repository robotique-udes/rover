#include "video_recording.hpp"

// Start the video recording and initialize all openCV members
bool Recording::startRecording()
{
    // Getting the directory for the recording
    std::string filePath = videoFolderPath_ + "/" + filename_;
    filePath.insert(filePath.length() - 4, '_' + std::to_string(this->recordingNumber++));  // add recording number before .avi
    this->files.push_back(filePath);                                                        // add file to list of recordings

    pipeline_
        = "rtspsrc location=" + camURL_
          + " latency=0 drop=true ! decodebin ! videorate max-rate=30 ! videoconvert ! queue max-size-buffers=1 ! appsink";

    this->cap.open(pipeline_, cv::CAP_GSTREAMER);
    if (!this->cap.isOpened())
    {
        RCLCPP_ERROR(*logger_, "Failed to open camera stream.");
        return false;
    }

    // Get frame width and height

    this->frame_width = static_cast<int>(this->cap.get(cv::CAP_PROP_FRAME_WIDTH));
    this->frame_height = static_cast<int>(this->cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    fps_ = static_cast<double>(this->cap.get(cv::CAP_PROP_FPS));

    fps_ = (fps_ > 0) ? fps_ : 30; 

    RCLCPP_DEBUG(*logger_, "fps set to %f", fps_);

    // Define the codec and create a VideoWriter object
    /* More information on OpenCV --> https://docs.opencv.org/4.x/dd/d9e/classcv_1_1VideoWriter.html */

    this->video_writer.open(filePath,
                            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                            fps_,
                            cv::Size(frame_width, frame_height));

    if (!video_writer.isOpened())
    {
        RCLCPP_ERROR(*logger_, "Error: Could not open the output video file for writing!");
        return false;
    }

    std::string appendedVideoFilePath = videoFolderPath_ + "/" + filename_;

    appender = cv::VideoWriter(appendedVideoFilePath,
                               cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                               fps_,
                               cv::Size(this->frame_width, this->frame_height));

    if (!appender.isOpened())
    {
        RCLCPP_ERROR(*logger_, "Couldn't launch video sticher");
        return false;
    }

    this->startTime = time(0);

    recordingThread = std::make_shared<std::thread>(
        [this]()
        {
            RecordingThreadFunction();
        });

    RCLCPP_INFO(*logger_, "Recording started for stream %s", camURL_.c_str());

    return true;
}

// Capture 1 frame and write it to the short save and long save
// Called from the recordingThread
bool Recording::recordFrame()
{
    if (!this->cap.isOpened())
    {
        if (!this->stopRecording.load())
            RCLCPP_ERROR(*logger_, "Error: cap is closed");
        return false;
    }

    this->cap >> frame_;
    if (this->frame_.empty())
    {
        if (!this->stopRecording.load())
            RCLCPP_ERROR(*logger_, "Error: Blank frame grabbed!");
        return false;
    }

    if (!this->video_writer.isOpened() || !this->appender.isOpened())
    {
        if (!this->stopRecording.load())
            RCLCPP_ERROR(*logger_, "Error: video writer is closed");
        return false;
    }
    // Write frame to the output video file
    this->video_writer.write(frame_);
    this->appender.write(frame_);

    // Show the frame
    // cv::imshow("IP Camera Stream", this->frame_);

    if (difftime(time(0), this->startTime) >= RECORDING_INTERVAL)  // save every RECORDING_INTERVAL seconds
    {
        std::string filePath = videoFolderPath_ + "/" + filename_;
        filePath.insert(filePath.length() - 4, '_' + std::to_string(this->recordingNumber++));
        this->files.push_back(filePath);
        this->video_writer.release();

        this->video_writer.open(filePath,
                                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                fps_,
                                cv::Size(this->frame_width, this->frame_height));

        if (!video_writer.isOpened())
        {
            RCLCPP_ERROR(*logger_, "Error: Could not open the output video file for writing!");
            return false;
        }

        this->startTime = time(0);
    }

    return true;
}

// call recordFrame and use callback function (shutdown request) in case of error
void Recording::RecordingThreadFunction()
{
    while (!this->stopRecording.load())
    {
        if (!recordFrame() && !this->stopRecording.load())  // if error execept on last loop
        {
            RCLCPP_WARN(*logger_, "Requesting shutdown for %s", camURL_.c_str());
            RequestShutdown_(camURL_);
            this->stopRecording.store(true);
        }
    }
    return;
}