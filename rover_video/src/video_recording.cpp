#include "video_recording.hpp"

// Start the video recording and initialize all openCV members
bool Recording::startRecording(void)
{
    // Getting the directory for the recording
    std::string filePath = _videoFolderPath + "/" + _filename;
    filePath.insert(filePath.length() - 4, '_' + std::to_string(_recordingNumber++));  // add recording number before .avi
    _files.push_back(filePath);                                                        // add file to list of recordings

    _pipeline = "rtspsrc location=" + _camURL
                + " latency=0 drop=true ! decodebin ! videorate max-rate=30 ! videoconvert ! queue max-size-buffers=1 ! appsink";

    _cap.open(_pipeline, cv::CAP_GSTREAMER);
    if (!_cap.isOpened())
    {
        RCLCPP_ERROR(*rLogger, "Failed to open camera stream.");
        return false;
    }

    // Get frame width and height

    _frame_width = static_cast<int>(_cap.get(cv::CAP_PROP_FRAME_WIDTH));
    _frame_height = static_cast<int>(_cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    _fps = static_cast<double>(_cap.get(cv::CAP_PROP_FPS));

    _fps = (_fps > 0) ? _fps : 30;

    RCLCPP_DEBUG(*rLogger, "fps set to %f", _fps);

    // Define the codec and create a VideoWriter object
    /* More information on OpenCV --> https://docs.opencv.org/4.x/dd/d9e/classcv_1_1VideoWriter.html */

    _video_writer.open(filePath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), _fps, cv::Size(_frame_width, _frame_height));

    if (!_video_writer.isOpened())
    {
        RCLCPP_ERROR(*rLogger, "Error: Could not open the output video file for writing!");
        return false;
    }

    std::string appendedVideoFilePath = _videoFolderPath + "/" + _filename;

    _appender = cv::VideoWriter(appendedVideoFilePath,
                                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                _fps,
                                cv::Size(_frame_width, _frame_height));

    if (!_appender.isOpened())
    {
        RCLCPP_ERROR(*rLogger, "Couldn't launch video sticher");
        return false;
    }

    _startTime = time(0);

    _recordingThread = std::thread(
        [this]()
        {
            recordingThreadFunction();
        });

    RCLCPP_INFO(*rLogger, "Recording started for stream %s", _camURL.c_str());

    return true;
}

// Capture 1 frame and write it to the short save and long save
// Called from the recordingThread
bool Recording::recordFrame(void)
{
    if (!_cap.isOpened())
    {
        if (!_stopRecording.load())
            RCLCPP_ERROR(*rLogger, "Error: cap is closed");
        return false;
    }

    _cap >> _frame;
    if (_frame.empty())
    {
        if (!_stopRecording.load())
            RCLCPP_ERROR(*rLogger, "Error: Blank frame grabbed!");
        return false;
    }

    if (!_video_writer.isOpened() || !_appender.isOpened())
    {
        if (!_stopRecording.load())
            RCLCPP_ERROR(*rLogger, "Error: video writer is closed");
        return false;
    }
    // Write frame to the output video file
    _video_writer.write(_frame);
    _appender.write(_frame);

    // Show the frame
    // cv::imshow("IP Camera Stream", this->_frame);

    if (difftime(time(0), _startTime) >= RECORDING_INTERVAL)  // save every RECORDING_INTERVAL seconds
    {
        std::string filePath = _videoFolderPath + "/" + _filename;
        filePath.insert(filePath.length() - 4, '_' + std::to_string(_recordingNumber++));
        _files.push_back(filePath);
        _video_writer.release();

        _video_writer.open(filePath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), _fps, cv::Size(_frame_width, _frame_height));

        if (!_video_writer.isOpened())
        {
            RCLCPP_ERROR(*rLogger, "Error: Could not open the output video file for writing!");
            return false;
        }

        _startTime = time(0);
    }

    return true;
}

// call recordFrame and use callback function (shutdown request) in case of error
void Recording::recordingThreadFunction(void)
{
    while (!_stopRecording.load())
    {
        if (!recordFrame() && !_stopRecording.load())  // if error execept on last loop
        {
            RCLCPP_WARN(*rLogger, "Requesting shutdown for %s", _camURL.c_str());
            _RequestShutdown(_camURL);
            _stopRecording.store(true);
        }
    }
    return;
}