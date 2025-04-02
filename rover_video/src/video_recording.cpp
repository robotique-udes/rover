#include "video_recording.hpp"

// Start the video recording and initialize all openCV members
bool Recording::startRecording()
{
    // Getting the directory for the recording
    std::string filePath = this->videoFolderPath + "/" + this->filename;
    filePath.insert(filePath.length() - 4, '_' + std::to_string(this->recordingNumber++));  // add recording number before .avi
    this->files.push_back(filePath);                                                        // add file to list of recordings

    this->pipeline
        = "rtspsrc location=" + this->camURL
          + " latency=0 drop=true ! decodebin ! videorate max-rate=30 ! videoconvert ! queue max-size-buffers=1 ! appsink";

    this->cap.open(this->pipeline, cv::CAP_GSTREAMER);
    if (!this->cap.isOpened())
    {
        RCLCPP_ERROR(*logger_, "Failed to open camera stream.");
        return false;
    }

    // Get frame width and height

    this->frame_width = static_cast<int>(this->cap.get(cv::CAP_PROP_FRAME_WIDTH));
    this->frame_height = static_cast<int>(this->cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    this->fps = static_cast<double>(this->cap.get(cv::CAP_PROP_FPS));

    this->fps = (this->fps > 0) ? fps : 30;  // weird bug with usb camera, recording is 2x speed or 1,5x

    RCLCPP_DEBUG(*logger_, "fps set to %f", this->fps);

    // Define the codec and create a VideoWriter object
    /* More information on OpenCV --> https://docs.opencv.org/4.x/dd/d9e/classcv_1_1VideoWriter.html */

    this->video_writer.open(filePath,
                            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                            this->fps,
                            cv::Size(frame_width, frame_height));

    if (!video_writer.isOpened())
    {
        RCLCPP_ERROR(*logger_, "Error: Could not open the output video file for writing!");
        return false;
    }

    std::string appendedVideoFilePath = this->videoFolderPath + "/" + this->filename;

    appender = cv::VideoWriter(appendedVideoFilePath,
                               cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                               this->fps,
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

    RCLCPP_INFO(*logger_, "Recording started for stream %s", camURL.c_str());

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

    this->cap >> this->frame;
    if (this->frame.empty())
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
    this->video_writer.write(this->frame);
    this->appender.write(this->frame);

    // Show the frame
    // cv::imshow("IP Camera Stream", this->frame);

    if (difftime(time(0), this->startTime) >= RECORDING_INTERVAL)  // save every RECORDING_INTERVAL seconds
    {
        std::string filePath = this->videoFolderPath + "/" + this->filename;
        filePath.insert(filePath.length() - 4, '_' + std::to_string(this->recordingNumber++));
        this->files.push_back(filePath);
        this->video_writer.release();

        this->video_writer.open(filePath,
                                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                this->fps,
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
            RCLCPP_WARN(*logger_, "Requesting shutdown for %s", this->camURL.c_str());
            RequestShutdown_(this->camURL);
            this->stopRecording.store(true);
        }
    }
    return;
}