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
                     rclcpp::Logger logger,
                     std::function<void(std::string)> RequestShutdown):
    _requestShutdown{RequestShutdown},
    _camURL{URL_in},
    _filename{filename_in},
    _videoFolderPath{videoFolderPath_in},
    rLogger{logger}
{
}

/**
 * @brief Construct a new Recording:: Recording object using move
 * @brief Used when emplacing temporary objects into a hashmap
 *
 * @param other Recording object
 */
Recording::Recording(Recording&& other):
    _requestShutdown(std::move(other._requestShutdown)),
    _recordingThread(std::move(other._recordingThread)),
    _camURL(std::move(other._camURL)),
    _pipeline(std::move(other._pipeline)),
    _filename(std::move(other._filename)),
    _videoFolderPath(std::move(other._videoFolderPath)),
    _files(std::move(other._files)),
    _recordingNumberShort(other._recordingNumberShort),
    _shortTimer(other._shortTimer),
    _longTimer(other._longTimer),
    _frameWidth(other._frameWidth),
    _frameHeight(other._frameHeight),
    _fps(other._fps),
    rLogger(other.rLogger),
    _cap(std::move(other._cap)),
    _video_writer_short(std::move(other._video_writer_short)),
    _video_writer_long(std::move(other._video_writer_long)),
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
Recording& Recording::operator=(Recording&& other)
{  // move operator just to be safe
    if (this != &other)
    {  // Prevent self-assignment

        // Move resources
        _requestShutdown = std::move(other._requestShutdown);
        _recordingThread = std::move(other._recordingThread);
        _stopRecording.store(other._stopRecording.load(std::memory_order_acquire), std::memory_order_release);

        _camURL = std::move(other._camURL);
        _pipeline = std::move(other._pipeline);
        _filename = std::move(other._filename);
        _videoFolderPath = std::move(other._videoFolderPath);
        _files = std::move(other._files);
        _recordingNumberShort = other._recordingNumberShort;
        _shortTimer = other._shortTimer;
        _longTimer = other._longTimer;
        _frameWidth = other._frameWidth;
        _frameHeight = other._frameHeight;
        _fps = other._fps;
        rLogger = other.rLogger;

        _video_writer_long = std::move(other._video_writer_long);
        _cap = std::move(other._cap);  // Move cv ressources
        _video_writer_short = std::move(other._video_writer_short);
        _frame = std::move(other._frame);
    }
    return *this;
}

/**
 * @brief Destroy the Recording:: Recording object.
 * @brief Handle the release of CV objects
 *
 */
Recording::~Recording(void)
{
    _stopRecording.store(true);

    if (_recordingThread.joinable())
    {
        _recordingThread.join();
        RCLCPP_INFO(rLogger, "Recording stopped.");
    }
    // Release resources
    _cap.release();
    _video_writer_short.release();
    _video_writer_long.release();
}

std::string Recording::getURL(void) const
{
    return _camURL;
}

/**
 * @brief Return filename for error logging
 *
 * @return std::string
 */
std::string Recording::getFilename(void) const
{
    return _filename;
}

cv::Mat Recording::getFrame(void) const
{
    return _frame;
}

double Recording::getFPS(void) const
{
    return _fps;
}

/**
 * @brief initialize all CV variables
 *
 * @return true if all CV variables are initialized correctly.
 * @return false if there's any error
 */
bool Recording::startRecording(void)
{
    // Getting the directory for the recording
    std::string filepath_short = _videoFolderPath + "/" + _filename;
    filepath_short.insert(filepath_short.length() - 4,
                          "_short_" + std::to_string(_recordingNumberShort++));  // add recording number before .avi
    _files.push_back(filepath_short);                                            // add file to list of recordings

    std::string filepath_long = _videoFolderPath + "/" + _filename;
    filepath_long.insert(filepath_long.length() - 4,
                         "_long_" + std::to_string(_recordingNumberLong++));  // add recording number before .avi

    _pipeline = "rtspsrc location=" + _camURL
                + " latency=0 drop=true ! decodebin ! videorate max-rate=30 ! videoconvert ! queue max-size-buffers=1 ! appsink";

    std::future<bool> opened = std::async(std::launch::async,
                                          [this]()
                                          {
                                              _cap.open(_pipeline, cv::CAP_GSTREAMER);
                                              return _cap.isOpened();
                                          });
    if (opened.wait_for(std::chrono::milliseconds(2000)) == std::future_status::ready)
    {
        if (!opened.get())
        {
            RCLCPP_ERROR(rLogger, "Failed to open camera stream.");
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(rLogger, "Timeout while opening camera stream.");
        return false;
    }

    // Get frame width and height

    _frameWidth = static_cast<int>(std::round(_cap.get(cv::CAP_PROP_FRAME_WIDTH)));
    _frameHeight = static_cast<int>(std::round(_cap.get(cv::CAP_PROP_FRAME_HEIGHT)));
    _fps = _cap.get(cv::CAP_PROP_FPS);

    _fps = CONSTRAIN(_fps, 0.0, 30.0);

    RCLCPP_DEBUG(rLogger, "fps set to %f", _fps);

    // Define the codec and create a VideoWriter object
    /* More information on OpenCV --> https://docs.opencv.org/4.x/dd/d9e/classcv_1_1VideoWriter.html */

    _video_writer_short.open(filepath_short, CODEC_MJPG, _fps, cv::Size(_frameWidth, _frameHeight));

    if (!_video_writer_short.isOpened())
    {
        RCLCPP_ERROR(rLogger, "Error: Could not open the output video file for writing short video!");
        return false;
    }

    _shortTimer = time(0);

    _video_writer_long = cv::VideoWriter(filepath_long, CODEC_MJPG, _fps, cv::Size(_frameWidth, _frameHeight));

    if (!_video_writer_long.isOpened())
    {
        RCLCPP_ERROR(rLogger, "Error: Could not open the output video file for writing long video!");
        return false;
    }

    _longTimer = time(0);
    _recordingThread = std::thread(
        [this](void)
        {
            recordingThreadFunction();
        });

    RCLCPP_INFO(rLogger, "Recording started for stream %s", _camURL.c_str());

    return true;
}

/**
 * @brief This function records 1 frame and writes in the short and the long video
 *
 * @return true if the read-write is a success.
 * @return false if there's any error
 */
bool Recording::recordFrame(void)
{
    if (!_cap.isOpened())
    {
        if (!_stopRecording.load())
        {
            RCLCPP_ERROR(rLogger, "Error: cap is closed");
        }
        return false;
    }

    _cap >> _frame;
    if (_frame.empty())
    {
        if (!_stopRecording.load())
        {
            RCLCPP_ERROR(rLogger, "Error: Blank frame grabbed!");
        }
        return false;
    }

    if (!_video_writer_short.isOpened() || !_video_writer_long.isOpened())
    {
        if (!_stopRecording.load())
            RCLCPP_ERROR(rLogger, "Error: video writer is closed");
        return false;
    }
    // Write frame to the output video file
    _video_writer_short.write(_frame);
    _video_writer_long.write(_frame);

    // Show the frame
    // cv::imshow("IP Camera Stream", this->_frame);

    if (difftime(time(0), _shortTimer) >= RECORDING_INTERVAL_SHORT_S)  // save every RECORDING_INTERVAL_SHORT_S seconds
    {
        std::string filepath_short = _videoFolderPath + "/" + _filename;
        filepath_short.insert(filepath_short.length() - 4, "_short_" + std::to_string(_recordingNumberShort++));
        _files.push_back(filepath_short);
        _video_writer_short.release();

        _video_writer_short.open(filepath_short, CODEC_MJPG, _fps, cv::Size(_frameWidth, _frameHeight));

        if (!_video_writer_short.isOpened())
        {
            RCLCPP_ERROR(rLogger, "Error: Could not open the output video file for writing!");
            return false;
        }

        _shortTimer = time(0);
    }

    if (difftime(time(0), _longTimer) >= RECORDING_INTERVAL_LONG_S)  // save every RECORDING_INTERVAL_LONG_S seconds
    {
        std::string filepath_long = _videoFolderPath + "/" + _filename;
        filepath_long.insert(filepath_long.length() - 4, "_long_" + std::to_string(_recordingNumberLong++));
        _video_writer_long.release();

        _video_writer_long.open(filepath_long, CODEC_MJPG, _fps, cv::Size(_frameWidth, _frameHeight));

        if (!_video_writer_long.isOpened())
        {
            RCLCPP_ERROR(rLogger, "Error: Could not open the output video file for writing!");
            return false;
        }

        _longTimer = time(0);
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
            RCLCPP_WARN(rLogger, "Requesting shutdown for %s", _camURL.c_str());
            _requestShutdown(_camURL);
            _stopRecording.store(true);
        }
    }
    return;
}