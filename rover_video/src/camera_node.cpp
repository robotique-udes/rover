#include "camera_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("media_server"), "Node initialized.");

    rclcpp::spin(std::make_shared<CameraNode>());

    rclcpp::shutdown();
    return 0;
}

/**
 * @brief Construct a new CameraNode object
 *
 */
CameraNode::CameraNode():
    Node("media_server")
{
    _srv_control = this->create_service<rover_msgs::srv::CameraControl>(
        "/rover/video/media_server",
        [this](const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request_,
               std::shared_ptr<rover_msgs::srv::CameraControl::Response> response_)
        {
            if (!request_ || !response_)
            {
                RCLCPP_ERROR(this->get_logger(), "NULL request or response received.");
                response_->success = false;
                response_->status = "Service call with null request or response. Possible internal ROS2 error.";
                return;
            }
            this->controlIPCam(*request_, *response_);
        });

    _sub_position
        = this->create_subscription<rover_msgs::msg::GpsPosition>("/rover/gps/position",
                                                                  1,
                                                                  [this](const rover_msgs::msg::GpsPosition& gps_message_)
                                                                  {
                                                                      this->callbackPosition(gps_message_);
                                                                  });
}

/**
 * @brief Decides what to do depending on the oncoming request
 *
 * @param request_
 * @param response_
 */
void CameraNode::controlIPCam(const rover_msgs::srv::CameraControl::Request& request_,
                              rover_msgs::srv::CameraControl::Response& response_)
{
    RCLCPP_DEBUG(this->get_logger(),
                 "Entry CameraNode::controlIPCam("
                 "resquest_.camera_url = %s\n"
                 "\t.capture_name = %s\n"
                 "\t.command = %u)\n"
                 "\tresponse: Not set",
                 request_.camera_url.c_str(),
                 request_.capture_name.c_str(),
                 request_.command);

    std::string folderPath;
    std::string captureName;
    std::string cameraURL = request_.camera_url;

    switch (request_.command)
    {
        case rover_msgs::srv::CameraControl::Request::TAKE_PICTURE:
            this->takeScreenshot(request_, response_);
            break;

        case rover_msgs::srv::CameraControl::Request::START_RECORDING:
            this->startRecordingLogic(request_, response_);
            break;

        case rover_msgs::srv::CameraControl::Request::STOP_RECORDING:
            this->stopRecordingLogic(request_, response_);
            break;

        default:
            RCLCPP_INFO(LOGGER, "Invalid command.");
            response_.success = false;
            response_.status = "Invalid command.";
            break;
    }
}

/**
 * @brief Executes the program in order to take a screenshot
 *
 * @param request_
 * @param response_
 */
void CameraNode::takeScreenshot(const rover_msgs::srv::CameraControl::Request& request_,
                                rover_msgs::srv::CameraControl::Response& response_)
{
    std::string folderPath;
    std::string captureName;
    std::string cameraURL = request_.camera_url;

    captureName = this->getFileName(request_.capture_name, cameraURL, eFileFormatNameTypes::SCREENSHOT);
    folderPath = this->getFolderPath(eFileFormatNameTypes::SCREENSHOT);

    if (!this->createFolder(folderPath))
    {
        RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
        response_.success = false;
        response_.status = "Failed to create screenshots folder or it already exists.";
        // Need to find how to handle
    }

    if (this->getScreenshot(folderPath, captureName, cameraURL))
    {
        response_.success = true;
        response_.status = "Screenshot saved as " + folderPath + "/" + captureName;
    }
    else
    {
        response_.success = false;
        response_.status = "Failed to take a screenshot.";
    }
}

/**
 * @brief Executes the program in order when a new recording starts
 *
 * @param request_
 * @param response_
 */
void CameraNode::startRecordingLogic(const rover_msgs::srv::CameraControl::Request& request_,
                                     rover_msgs::srv::CameraControl::Response& response_)
{
    std::string folderPath;
    std::string captureName;
    std::string cameraURL = request_.camera_url;

    captureName = this->getFileName(request_.capture_name, cameraURL, eFileFormatNameTypes::VIDEO);
    folderPath = this->getFolderPath(eFileFormatNameTypes::VIDEO);
    if (!this->createFolder(folderPath))
    {
        RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
        response_.success = false;
        response_.status = "Failed to create screenshots folder or it already exists.";
        // Need to find how to handle
    }
    if (this->newRecording(folderPath, captureName, cameraURL))
    {
        response_.success = true;
        response_.status = "Recording started";
    }
    else
    {
        response_.success = false;
        response_.status = "Failed to take a video.";  // add reason i.e. recording already started at TIME-GPS-NAME
    }
}

/**
 * @brief Executes the program in order when a recording stops
 *
 * @param request_
 * @param response_
 */
void CameraNode::stopRecordingLogic(const rover_msgs::srv::CameraControl::Request& request_,
                                    rover_msgs::srv::CameraControl::Response& response_)
{
    std::string cameraURL = request_.camera_url;

    if (this->stopRecording(cameraURL))
    {
        response_.success = true;
        response_.status = "Recording ended";
    }
    else
    {
        response_.success = false;
        response_.status = "No such recordings";
    }
}

/**
 * @brief Get the time the request was made
 *
 * @return std::string
 */
std::string CameraNode::getCurrentTime(void)
{
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();  // get system time

    std::time_t now_time = std::chrono::system_clock::to_time_t(now);  // convert to real time

    std::tm tm_now = *std::localtime(&now_time);  // convert to calendar time

    std::stringstream current_time_output;
    current_time_output << std::put_time(&tm_now, "%FT%T");  // ISO 8601 format

    return current_time_output.str();
}

/**
 * @brief Gets the filename necessary to save the file, depending on the file type
 *
 * @param capture_name_ Custom name made by the user
 * @param camURL_ The rtsp url for the camera in use
 * @param fileType_ Whether it is a screenshot or a video
 * @return std::string of the complete filename
 */
std::string CameraNode::getFileName(const std::string& capture_name_, std::string camURL_, eFileFormatNameTypes fileType_)
{
    std::string filename;

    std::string time = this->getCurrentTime();
    std::string latitude = std::to_string(_last_latitude);
    std::string longitude = std::to_string(_last_longitude);
    std::string ID = "UnknownID";
    auto it = CameraInfo::CameraName.find(camURL_);

    if (it != CameraInfo::CameraName.end())
    {
        ID = it->second;
    }

    switch (fileType_)
    {
        case eFileFormatNameTypes::SCREENSHOT:
            filename = capture_name_.empty()
                           ? time + "_lat:" + latitude + "_long:" + longitude + "_" + ID + "_screenshot.png"
                           : time + "_lat:" + latitude + "_long:" + longitude + "_camID:" + ID + "_" + capture_name_;
            // Example : 2024-12-10T20:50:00_GPS_30_screenshot.png
            break;

        case eFileFormatNameTypes::VIDEO:
            filename = capture_name_.empty()
                           ? time + "_lat:" + latitude + "_long:" + longitude + "_" + ID + "_recording.avi"
                           : time + "_lat:" + latitude + "_long:" + longitude + "_camID:" + ID + "_" + capture_name_;
            // Example : 2024-12-10T20:50:00_GPS_30_recording.avi
            break;
    }
    return filename;
}

/**
 * @brief Finds the directory of the current package in use depending on the file type
 *
 * @param fileType_ Whether it is a screenshot or a video
 * @return const std::string of the complete directory
 */
const std::string CameraNode::getFolderPath(eFileFormatNameTypes fileType_)
{
    std::string folderPath;
    std::string currentPackageDirectory = GET_PACKAGE_SOURCE_DIR("rover_video");  // finds the path to our package

    switch (fileType_)
    {
        case eFileFormatNameTypes::SCREENSHOT:
            folderPath = std::string(currentPackageDirectory) + "/src/screenshots";
            break;

        case eFileFormatNameTypes::VIDEO:
            folderPath = std::string(currentPackageDirectory) + "/src/recordings";
            break;
    }

    return folderPath;
}

/**
 * @brief Checks if the screenshot or the recording folder exists
 *
 * @param path_ Path the the saving folder
 * @return true if it exists.
 * @return false if it doesn't or it isn't a folder
 */
bool CameraNode::folderExists(const std::string& path_)
{
    struct stat fileInfo;

    if (stat(path_.c_str(), &fileInfo) != 0)
    {
        return false;
    }

    if (fileInfo.st_mode & S_IFDIR)
    {
        return true;
    }
    else
    {
        RCLCPP_FATAL(this->get_logger(), "Element already exist with this path and name, but isn't a folder");
        assert(false);
    }
}

/**
 * @brief Creates the desired folder with the necessary permissions for Linux
 *
 * @param path_ Path to the folder that needs to be created
 * @return true
 * @return false
 */
bool CameraNode::createFolder(const std::string& path_)
{
    if (!this->folderExists(path_))
    {
        if (mkdir(path_.c_str(), 0775) == 0)
        {  // 0775 = Permissions for Linux
            RCLCPP_INFO(LOGGER, "Succesfully created the folder.");
            return true;
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Couldn't create the folder.");
            return false;
        }
    }

    RCLCPP_DEBUG(LOGGER, "Directory already exists: %s", path_.c_str());
    return true;
}

/**
 * @brief OpenCV implementation in order to capture a screenshot from a desired camera
 *
 * @param screenshotFolderPath_ Absolute path to the saving folder
 * @param filename_ The name of the resulting screenshot
 * @param cameraURL_ RTSP url of the camera that is currently being used
 * @return true if succesfully taken a screenshot.
 * @return false if unsuccesful in its task
 */
bool CameraNode::getScreenshot(std::string screenshotFolderPath_, std::string filename_, std::string cameraURL_)
{
    std::string captureName = screenshotFolderPath_ + "/" + filename_;

    RCLCPP_INFO(LOGGER, "Attempting to capture screenshot from camera: %s", cameraURL_.c_str());

    // The URL format will depend on the camera model and configuration
    // std::string camera_url = "rtsp://rover:roverrover@192.168.144.30:554/1/h264major";

    // Open the video stream

    if (_RecordingMap.find(cameraURL_) != _RecordingMap.end())  // check if currently recording
    {
        Recording* pRecording = &_RecordingMap.at(cameraURL_);

        // Save the last frame from recording as picture:
        cv::imwrite(captureName, pRecording->getFrame());
        RCLCPP_INFO(LOGGER, "Screenshot saved successfully as: %s", captureName.c_str());

        // Display the frame for debug
        // cv::imshow("IP Camera Screenshot", pRecording->getFrame());

        return true;
    }
    else  // if not recording proceed normaly
    {
        std::string pipeline
            = "rtspsrc location=" + cameraURL_
              + " latency=0 drop=true ! decodebin ! videorate max-rate=30 ! videoconvert ! queue max-size-buffers=1 ! appsink";

        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

        if (!cap.isOpened())
        {
            RCLCPP_ERROR(LOGGER, "Failed to open camera stream.");
            return false;
        }

        // Read a single frame
        cv::Mat frame;
        bool ret = cap.read(frame);

        if (ret)
        {
            // Save the frame as a sceenshot:
            cv::imwrite(captureName, frame);
            RCLCPP_INFO(LOGGER, "Screenshot saved successfully as: %s", captureName.c_str());

            // Display the frame for debug
            // cv::imshow("IP Camera Screenshot", frame);
            // cv::waitKey(0);  // Wait for a key press
            // cv::destroyAllWindows();
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Failed to capture frame from camera.");
            return false;
        }

        // Release the video capture object
        cap.release();

        return true;
    }
}

/**
 * @brief Stops a recording that is being currently made
 *
 * @param cameraURL_ RTSP url of the camera that is currently being used
 * @return true if successfully stopped the recording.
 * @return false if unsuccesful
 */
bool CameraNode::stopRecording(std::string cameraURL_)
{
    std::lock_guard<std::mutex> lock(_recordingMutex);

    if (_RecordingMap.find(cameraURL_) != _RecordingMap.end())
    {
        _RecordingMap.erase(cameraURL_);

        if (_RecordingMap.empty())
        {
            _watchDogStop.store(true);
            _recordingCv.notify_one();
        }
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Creates a new recording inside the hashmap if it doesn't already exists.
 *
 * @param videoFolderPath_ Absolute path to the saving folder
 * @param filename_ Name for the recording
 * @param cameraURL_ RTSP url of the camera that is currently being used
 * @return true
 * @return false
 */
bool CameraNode::newRecording(std::string videoFolderPath_, std::string filename_, std::string cameraURL_)
{
    if (_RecordingMap.find(cameraURL_) != _RecordingMap.end())  // check if recording doesn't already exist
    {
        return false;
    }
    else
    {
        _RecordingMap.emplace(cameraURL_,
                              Recording(videoFolderPath_,
                                        filename_,
                                        cameraURL_,
                                        std::make_shared<rclcpp::Logger>(LOGGER),
                                        [this](std::string url)
                                        {
                                            RequestShutdown(url);
                                        }));

        if (!_videoWatchDog.joinable())
        {
            StartWatchDog();
        }

        // Access the recording using at() to safely get the reference
        Recording* pRecording = &_RecordingMap.at(cameraURL_);

        if (pRecording->startRecording())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

/**
 * @brief Writes the current value of the GPS onto placeholding variables
 *
 * @param gps_message_ Address reference of the GPS subscriber
 */
void CameraNode::callbackPosition(const rover_msgs::msg::GpsPosition& gps_message_)
{
    _last_latitude = gps_message_.latitude;
    _last_longitude = gps_message_.longitude;
}

/**
 * @brief Add the URL to the shutdown list and notify watchdog to process shutdown
 *        This function is passed as a callback function to the recording class
 * @param camURL_ key for the hashmap
 */
void CameraNode::RequestShutdown(std::string camURL_)
{
    RCLCPP_WARN(LOGGER, "Received shutdown request for %s", camURL_.c_str());

    {
        std::unique_lock<std::mutex> lock(_recordingMutex);
        _RecordingShutdownRequestSet.insert(camURL_);
    }  // unlock
    _recordingCv.notify_one();
    return;
}

/**
 * @brief Start the Watchdog thread
 *
 */
bool CameraNode::StartWatchDog(void)
{
    _watchDogStop.store(false);
    _videoWatchDog = std::thread(&CameraNode::VideoWatchDogFunction, this);
    return true;
}

/**
 * @brief Shutdown and erase a recording from the hashmap
 * @until watchDogStop = true
 */
void CameraNode::VideoWatchDogFunction(void)
{
    RCLCPP_DEBUG(LOGGER, "Starting video watchdog");
    while (!_watchDogStop.load())
    {
        std::unique_lock<std::mutex> lock(_recordingMutex);
        _recordingCv.wait(lock,
                          [this]
                          {
                              return _watchDogStop.load() || !_RecordingShutdownRequestSet.empty();
                          });

        if (_watchDogStop)
        {
            break;
        }
        else
        {
            for (std::string url : _RecordingShutdownRequestSet)
            {
                RCLCPP_WARN(LOGGER, "Processing Shutdown for %s", url.c_str());
                if (_RecordingMap.find(url) != _RecordingMap.end())
                {
                    if (!_RecordingMap.erase(url))
                    {
                        RCLCPP_ERROR(LOGGER, "Shutdown request for %s could not be processed, please try again", url.c_str());
                    }

                    if (_RecordingMap.empty())
                    {
                        _watchDogStop.store(true);
                    }
                }
                else
                {
                    RCLCPP_ERROR(LOGGER, "Unable to find %s for shutdown, please try again", url.c_str());
                }
            }

            _RecordingShutdownRequestSet.clear();
        }
    }
    RCLCPP_DEBUG(LOGGER, "Stopping video watchdog");
    return;
}