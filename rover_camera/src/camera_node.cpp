#include "camera_node.hpp"

#include <rover_lib2/helpers/constants.hpp>
#include <rover_lib2/helpers/folders.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CameraNode>());

    rclcpp::shutdown();
    return 0;
}

CameraNode::CameraNode():
    Node("media_server")
{
    _srv_control = this->create_service<rover_msgs::srv::CameraControl>(
        SERVICE_MEDIA_SERVER_NAME,
        [this](const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request_,
               std::shared_ptr<rover_msgs::srv::CameraControl::Response> response_)
        {
            if (!request_ || !response_)
            {
                RCLCPP_ERROR(this->get_logger(), "NULL request or response received.");
                return;
            }
            this->controlIPCam(*request_, *response_);
        });

    _pub_urls = this->create_publisher<rover_msgs::msg::CameraList>(TOPIC_MEDIA_SERVER_NAME, 1);

    _timer_pub = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_PERIOD_MS),
                                         [this](void)
                                         {
                                             this->CB_url_publisher();
                                         });

    _sub_position
        = this->create_subscription<rover_msgs::msg::Gps>(TOPIC_GPS_NAME,
                                                                  QOS_DEFAULT,
                                                                  [this](const rover_msgs::msg::Gps& gps_message_)
                                                                  {
                                                                      this->callbackPosition(gps_message_);
                                                                  });
}

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
            RCLCPP_WARN(this->get_logger(), "Invalid command.");
            response_.success = false;
            response_.status = "Invalid command.";
            break;
    }
}

void CameraNode::takeScreenshot(const rover_msgs::srv::CameraControl::Request& request_,
                                rover_msgs::srv::CameraControl::Response& response_)
{
    std::optional<std::string> folderPathOptional;
    std::string captureName;
    std::string cameraURL = request_.camera_url;
    std::string currentCamera;

    captureName = this->getFileName(request_.capture_name, cameraURL, eFileFormatNameTypes::SCREENSHOT);
    folderPathOptional = this->getFolderPath(request_.base_path, eFileFormatNameTypes::SCREENSHOT);
    Constants::CameraInfo::getNameFromURL(cameraURL, currentCamera);

    if (!folderPathOptional)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to find home environment when screenshoting camera %s", currentCamera.c_str());
        response_.success = false;
        response_.status = "Failed to find home environment for saving screenshot on camera: " + currentCamera;
        return;
    }

    std::string folderPath = folderPathOptional.value();

    if (!Folders::createFolder(folderPath))
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create screenshots folder or it already exists at %s for camera: %s",
                     folderPath.c_str(),
                     currentCamera.c_str());
        response_.success = false;
        response_.status
            = "Failed to create screenshots folder or it already exists at " + folderPath + " for camera: " + currentCamera;
    }

    sScreenshotResult screenshotResult = this->getScreenshot(folderPath, captureName, cameraURL);
    if (screenshotResult.success)
    {
        response_.success = true;
        response_.status = "Screenshot saved as " + folderPath + "/" + captureName;
    }
    else
    {
        response_.success = false;
        response_.status = "Failed to take a screenshot. " + screenshotResult.msg;
    }
}

void CameraNode::startRecordingLogic(const rover_msgs::srv::CameraControl::Request& request_,
                                     rover_msgs::srv::CameraControl::Response& response_)
{
    std::optional<std::string> folderPathOptional;
    std::string captureName;
    std::string cameraURL = request_.camera_url;
    std::string currentCamera;

    captureName = this->getFileName(request_.capture_name, cameraURL, eFileFormatNameTypes::VIDEO);
    folderPathOptional = this->getFolderPath(request_.base_path, eFileFormatNameTypes::VIDEO);
    Constants::CameraInfo::getNameFromURL(cameraURL, currentCamera);

    if (!folderPathOptional)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to find home environment when recording camera %s", currentCamera.c_str());
        response_.success = false;
        response_.status = "Failed to find home environment for saving recording on camera: " + currentCamera;
        return;
    }

    std::string folderPath = folderPathOptional.value();

    if (!Folders::createFolder(folderPath))
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create recordings folder or it already exists at %s for camera: %s",
                     folderPath.c_str(),
                     currentCamera.c_str());
        response_.success = false;
        response_.status
            = "Failed to create recordings folder or it already exists at " + folderPath + "for camera: " + currentCamera;
    }
    if (this->newRecording(folderPath, captureName, cameraURL))
    {
        response_.success = true;
        response_.status = "Recording started";
    }
    else
    {
        this->requestShutdown(cameraURL);
        response_.success = false;
        response_.status = "Failed to take a video, check logs for reason";
    }
}

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

std::string CameraNode::getCurrentTime(void)
{
    std::stringstream current_time_output;

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);  // convert to real time
    std::tm tm_now = *std::localtime(&now_time);                       // convert to calendar time
    current_time_output << std::put_time(&tm_now, "%FT%T");            // ISO 8601 format

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
    std::string latitude = std::to_string(_lastLatitude);
    std::string longitude = std::to_string(_lastLongitude);
    std::string ID = "UnknownID";
    Constants::CameraInfo::getNameFromURL(camURL_, ID);

    switch (fileType_)
    {
        case eFileFormatNameTypes::SCREENSHOT:
            filename = capture_name_.empty()
                           ? time + "_lat:" + latitude + "_long:" + longitude + "_" + ID + "_screenshot.png"
                           : time + "_lat:" + latitude + "_long:" + longitude + "_camID:" + ID + "_" + capture_name_ + ".png";
            // Example : 2024-12-10T20:50:00_GPS_30_screenshot.png
            break;

        case eFileFormatNameTypes::VIDEO:
            filename = capture_name_.empty()
                           ? time + "_lat:" + latitude + "_long:" + longitude + "_" + ID + "_recording.avi"
                           : time + "_lat:" + latitude + "_long:" + longitude + "_camID:" + ID + "_" + capture_name_ + ".avi";
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
std::optional<std::string> CameraNode::getFolderPath(const std::string& basePath_, eFileFormatNameTypes fileType_)
{
    const char* home = std::getenv("HOME");
    std::string homeStr;
    if (home)
    {
        homeStr = home;
    }
    else
    {
        return std::nullopt;
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to create session folder, $HOME env variable wasn't found");
    }

    std::string folderPath;
    const std::string pathForScreenshots = "/screenshots";
    const std::string pathForRecordings = "/recordings";

    switch (fileType_)
    {
        case eFileFormatNameTypes::SCREENSHOT:
            folderPath = homeStr + basePath_ + pathForScreenshots;
            break;

        case eFileFormatNameTypes::VIDEO:
            folderPath = homeStr + basePath_ + pathForRecordings;
            break;
    }

    return folderPath;
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
sScreenshotResult CameraNode::getScreenshot(std::string screenshotFolderPath_, std::string filename_, std::string cameraURL_)
{
    sScreenshotResult result{false, ""};
    std::string captureName = screenshotFolderPath_ + "/" + filename_;

    RCLCPP_INFO(this->get_logger(), "Attempting to capture screenshot from camera: %s", cameraURL_.c_str());

    // The URL format will depend on the camera model and configuration
    // std::string camera_url = "rtsp://usual:usualusual@192.168.144.30:554/1/h264major";

    // Open the video stream

    if (_recordingMap.find(cameraURL_) != _recordingMap.end())  // check if currently recording
    {
        std::lock_guard<std::mutex> lock(_recordingMapMutex);
        Recording& rRecording = _recordingMap.at(cameraURL_);

        // Save the last frame from recording as picture:
        cv::imwrite(captureName, rRecording.getFrame());
        RCLCPP_INFO(this->get_logger(), "Screenshot saved successfully as: %s", captureName.c_str());

        // Display the frame for debug
        // cv::imshow("IP Camera Screenshot", rRecording.getFrame());

        result.success = true;
        return result;
    }
    else  // if not recording proceed normaly
    {
        std::string pipeline
            = "rtspsrc location=" + cameraURL_
              + " latency=0 drop=true ! decodebin ! videorate max-rate=30 ! videoconvert ! queue max-size-buffers=1 ! appsink";

        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

        if (!cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera stream.");
            result.msg = "Unable to open rtsp pipeline (cap)";
            return result;
        }

        // Read a single frame
        cv::Mat frame;
        bool ret = cap.read(frame);

        if (ret)
        {
            // Save the frame as a sceenshot:
            cv::imwrite(captureName, frame);
            RCLCPP_INFO(this->get_logger(), "Screenshot saved successfully as: %s", captureName.c_str());

            // Display the frame for debug
            // cv::imshow("IP Camera Screenshot", frame);
            // cv::waitKey(0);  // Wait for a key press
            // cv::destroyAllWindows();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture frame from camera. (cap.read = 0)");
            cap.release();
            result.msg = "Couldn't read frame (cap.read = 0)";
            return result;
        }

        // Release the video capture object
        cap.release();

        result.success = true;
        return result;
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
    {
        std::lock_guard<std::mutex> lock(_recordingMapMutex);

        if (_recordingMap.find(cameraURL_) == _recordingMap.end())
        {
            return false;
        }

        _recordingMap.erase(cameraURL_);
        if (_recordingMap.empty())
        {
            _watchDogStop.store(true);
            _recordingCv.notify_one();
        }
    }
    this->CB_url_publisher();
    return true;
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
    {
        std::lock_guard<std::mutex> lock(_recordingMapMutex);
        if (_recordingMap.find(cameraURL_) != _recordingMap.end())
        {
            Recording& rRecording = _recordingMap.at(cameraURL_);
            RCLCPP_WARN(this->get_logger(), "Recording already exist!\nSee file:\t%s", rRecording.getFilename().c_str());
            return false;
        }
        else
        {
            _recordingMap.emplace(cameraURL_,
                                  Recording(videoFolderPath_,
                                            filename_,
                                            cameraURL_,
                                            this->get_logger(),
                                            [this](std::string url_)
                                            {
                                                this->requestShutdown(url_);
                                            }));

            if (!_videoThread.joinable())
            {
                startWatchDog();
            }

            Recording& rRecording = _recordingMap.at(cameraURL_);

            return rRecording.startRecording();
        }
    }
    CB_url_publisher();
}

/**
 * @brief Writes the current value of the GPS onto placeholding variables
 *
 * @param gps_message_ Address reference of the GPS subscriber
 */
void CameraNode::callbackPosition(const rover_msgs::msg::Gps& gps_message_)
{
    _lastLatitude = gps_message_.latitude;
    _lastLongitude = gps_message_.longitude;
}

/**
 * @brief Add the URL to the shutdown list and notify watchdog to process shutdown
 *        This function is passed as a callback function to the recording class
 * @param camURL_ key for the hashmap
 */
void CameraNode::requestShutdown(std::string camURL_)
{
    RCLCPP_WARN(this->get_logger(), "Received shutdown request for %s", camURL_.c_str());

    {
        std::unique_lock<std::mutex> lock(_recordingMapMutex);
        _recordingShutdownRequestSet.insert(camURL_);
    }  // unlock
    _recordingCv.notify_one();
    return;
}

/**
 * @brief Start the Watchdog thread
 *
 */
bool CameraNode::startWatchDog(void)
{
    _watchDogStop.store(false);
    _videoThread = std::thread(&CameraNode::videoWatchDogFunction, this);
    return true;
}

/**
 * @brief Shutdown and erase a recording from the hashmap
 * @until watchDogStop = true
 */
void CameraNode::videoWatchDogFunction(void)
{
    RCLCPP_DEBUG(this->get_logger(), "Starting video watchdog");
    while (!_watchDogStop.load())
    {
        {
            std::unique_lock<std::mutex> lock(_recordingMapMutex);
            _recordingCv.wait(lock,
                              [this]
                              {
                                  return (_watchDogStop.load() || !_recordingShutdownRequestSet.empty());
                              });

            if (_watchDogStop)
            {
                break;
            }
            else
            {
                for (std::string url : _recordingShutdownRequestSet)
                {
                    RCLCPP_WARN(this->get_logger(), "Processing Shutdown for %s", url.c_str());
                    if (_recordingMap.find(url) != _recordingMap.end())
                    {
                        if (!_recordingMap.erase(url))
                        {
                            RCLCPP_ERROR(this->get_logger(),
                                         "Shutdown request for %s could not be processed, please try again",
                                         url.c_str());
                        }

                        if (_recordingMap.empty())
                        {
                            _watchDogStop.store(true);
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(),
                                     "Shutdown requested for %s but no recordings found, no action done",
                                     url.c_str());
                    }
                }
            }
        }
        _recordingShutdownRequestSet.clear();
    }
    RCLCPP_DEBUG(this->get_logger(), "Stopping video watchdog");
    return;
}

void CameraNode::CB_url_publisher(void)
{
    rover_msgs::msg::CameraList msg;

    {
        std::lock_guard<std::mutex> lock(_recordingMapMutex);

        for (const auto& recording : _recordingMap)
        {
            msg.urls.push_back(recording.second.getURL());
        }
    }

    _pub_urls->publish(msg);
}
