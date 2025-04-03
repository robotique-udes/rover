#include "video_recording.hpp"


/**
 * @brief Construct a new Recording:: Recording object
 * 
 * @param videoFolderPath_in Path to the video folder
 * @param filename_in Desired file name
 * @param URL_in Target URL
 * @param logger CameraNode logger
 * @param RequestShutdown callback function
 * @attention URL must be to a RTSP stream for GStreamer pipeline
 */
Recording::Recording(std::string videoFolderPath_in,
                     std::string filename_in,
                     std::string URL_in,
                     std::shared_ptr<rclcpp::Logger> logger,
                     std::function<void(std::string)> RequestShutdown):
    _RequestShutdown(RequestShutdown),
    _camURL(URL_in),
    _filename(filename_in),
    _videoFolderPath(videoFolderPath_in),
    rLogger(logger)
{}



/**
 * @brief Construct a new Recording:: Recording object using move
 * @brief Used when emplacing temporary objects into a hashmap
 * 
 * @param other Recording object
 */
Recording::Recording(Recording&& other) noexcept:
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


/**
 * @brief Move operator for the recording class
 * 
 * @param other Recording object
 * @return Recording& 
 */
Recording& Recording::operator=(Recording&& other) noexcept
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

/**
 * @brief Destroy the Recording:: Recording object \n
 * @brief Handle the release of CV objects
 * 
 */
Recording::~Recording(void)
{
    if (_cap.isOpened())  // avoid unnecessary logging when creating temporary objects
    {
        _stopRecording.store(true);

        if (_recordingThread.joinable())
        {
            _recordingThread.join();
        }
        // Release resources
        _cap.release();
        _video_writer.release();
        _appender.release();

        RCLCPP_INFO(*rLogger, "Recording stopped.");
    }
}

/**
 * @brief initialize all CV variables
 * 
 * @return true if all CV variables are initialized correctly \n
 * @return false if there's any error
 */
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

/**
 * @brief This function records 1 frame and writes in the short and the long video
 * 
 * @return true if the read-write is a success \n
 * @return false if there's any error
 */
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

/**
 * @brief this function records frame in a separate thread
 * @exception When record frame returns 0, use shutdown callback function
 */
void Recording::recordingThreadFunction(void)
{
    while (!_stopRecording.load())
    {
        if (!this->recordFrame() && !_stopRecording.load())  // if error execept on last loop
        {
            RCLCPP_WARN(*rLogger, "Requesting shutdown for %s", _camURL.c_str());
            _RequestShutdown(_camURL);
            _stopRecording.store(true);
        }
    }
    return;
}