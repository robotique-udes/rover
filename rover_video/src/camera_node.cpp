#include "camera_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("media_server"), "Node initialized.");

    try
    {
        rclcpp::spin(std::make_shared<CameraNode>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("Dead Node"), "Killing node on exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

CameraNode::CameraNode():
    Node("media_server")
{
    _srv_control = this->create_service<rover_msgs::srv::CameraControl>(
        "/rover/video/media_server",
        [this](const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request,
               std::shared_ptr<rover_msgs::srv::CameraControl::Response> response)
        {
            if (!request || !response)
            {
                RCLCPP_ERROR(this->get_logger(), "NULL request or response received.");
                response->success = false;
                response->status = "Service call with null request or response. Possible internal ROS2 error.";
                return;
            }
            this->controlIPCam(*request, *response);
        });

    _msg_position
        = this->create_subscription<rover_msgs::msg::GpsPosition>("/rover/gps/position",
                                                                  1,
                                                                  [this](const rover_msgs::msg::GpsPosition& gps_message)
                                                                  {
                                                                      this->callbackPosition(gps_message);
                                                                  });
}

void CameraNode::controlIPCam(const rover_msgs::srv::CameraControl::Request& request,
                              rover_msgs::srv::CameraControl::Response& response)
{
    RCLCPP_DEBUG(LOGGER, "Entering the controlIPCam function");

    std::string folderPath;
    std::string captureName;
    std::string cameraURL = request.camera_url;

    switch (request.command)
    {
        case rover_msgs::srv::CameraControl::Request::TAKE_PICTURE:
            captureName = this->getFileName(request.capture_name, cameraURL, eFileFormatNameTypes::SCREENSHOT);
            folderPath = this->getFolderPath(eFileFormatNameTypes::SCREENSHOT);

            if (!this->createFolder(folderPath))
            {
                RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
                response.success = false;
                response.status = "Failed to create screenshots folder or it already exists.";
                break;
            }

            if (this->getScreenshot(folderPath, captureName, cameraURL))
            {
                response.success = true;
                response.status = "Screenshot saved as " + folderPath + "/" + captureName;
            }
            else
            {
                response.success = false;
                response.status = "Failed to take a screenshot.";
            }
            break;

        case rover_msgs::srv::CameraControl::Request::START_RECORDING:
            captureName = this->getFileName(request.capture_name, cameraURL, eFileFormatNameTypes::VIDEO);
            folderPath = this->getFolderPath(eFileFormatNameTypes::VIDEO);
            if (!this->createFolder(folderPath))
            {
                RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
                response.success = false;
                response.status = "Failed to create screenshots folder or it already exists.";
                break;
            }
            if (this->newRecording(folderPath, captureName, cameraURL))
            {
                response.success = true;
                response.status = "Recording started";
            }
            else
            {
                response.success = false;
                response.status = "Failed to take a video.";  // add reason i.e. recording already started at TIME-GPS-NAME
            }

            break;

        case rover_msgs::srv::CameraControl::Request::STOP_RECORDING:
            if (this->stopRecording(cameraURL))
            {
                response.success = true;
                response.status = "Recording ended";
            }
            else
            {
                response.success = false;
                response.status = "No such recordings";
            }
            break;

        default:
            RCLCPP_INFO(LOGGER, "Invalid command.");
            response.success = false;
            response.status = "Invalid command.";
            break;
    }
}

std::string CameraNode::getCamID(std::string cameraURL)
{
    std::string camID;

    std::string::size_type nextDotPos;
    std::string::size_type posID = cameraURL.find("144.");

    if (posID != std::string::npos)
    {
        RCLCPP_DEBUG(LOGGER, "'144.' found.");
        posID += 4;
        nextDotPos = cameraURL.find(':', posID);

        if (nextDotPos != std::string::npos)
        {
            camID = cameraURL.substr(posID, nextDotPos - posID);
        }
        else
        {
            camID = cameraURL.substr(posID);
        }
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "'144.' not found.");
    }

    RCLCPP_DEBUG(LOGGER, "Camera ID: %s", camID.c_str());

    return camID;
}

std::string CameraNode::getCurrentTime()
{
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();  // get system time

    std::time_t now_time = std::chrono::system_clock::to_time_t(now);  // convert to real time

    std::tm tm_now = *std::localtime(&now_time);  // convert to calendar time

    std::stringstream current_time_output;
    current_time_output << std::put_time(&tm_now, "%FT%T");  // ISO 8601 format

    return current_time_output.str();
}

std::string CameraNode::getFileName(const std::string& capture_name, std::string camURL, eFileFormatNameTypes state)
{
    std::string filename;

    std::string time = this->getCurrentTime();
    std::string latitude = std::to_string(last_latitude);
    std::string longitude = std::to_string(last_longitude);
    std::string ID = this->getCamID(camURL);

    switch (state)
    {
        case eFileFormatNameTypes::SCREENSHOT:
            filename = capture_name.empty()
                           ? time + "_lat:" + latitude + "_long:" + longitude + "_" + ID + "_screenshot.png"
                           : time + "_lat:" + latitude + "_long:" + longitude + "_camID:" + ID + "_" + capture_name;
            // Example : 2024-12-10T20:50:00_GPS_30_screenshot.png
            break;

        case eFileFormatNameTypes::VIDEO:
            filename = capture_name.empty()
                           ? time + "_lat:" + latitude + "_long:" + longitude + "_" + ID + "_recording.avi"
                           : time + "_lat:" + latitude + "_long:" + longitude + "_camID:" + ID + "_" + capture_name;
            // Example : 2024-12-10T20:50:00_GPS_30_recording.avi
            break;
    }
    return filename;
}

const std::string CameraNode::getFolderPath(eFileFormatNameTypes state)
{
    std::string folderPath;
    std::string currentPackageDirectory = GET_PACKAGE_SOURCE_DIR("rover_video");  // finds the path to our package

    switch (state)
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

bool CameraNode::folderExists(const std::string& path)
{
    struct stat fileInfo;

    if (stat(path.c_str(), &fileInfo) != 0)
    {
        return false;
    }

    if (fileInfo.st_mode & S_IFDIR)
    {
        return true;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Element Already exist with this path and name, but isn't a folder");
        return false;  // Todo: how to handle this request
    }
}

bool CameraNode::createFolder(const std::string& path)
{
    if (!this->folderExists(path))
    {
        if (mkdir(path.c_str(), 0775) == 0)
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

    RCLCPP_DEBUG(LOGGER, "Directory already exists: %s", path.c_str());
    return true;
}

bool CameraNode::getScreenshot(std::string screenshotFolderPath, std::string filename, std::string cameraURL)
{
    std::string captureName = screenshotFolderPath + "/" + filename;

    RCLCPP_INFO(LOGGER, "Attempting to capture screenshot from camera: %s", cameraURL.c_str());

    // The URL format will depend on the camera model and configuration
    // std::string camera_url = "rtsp://rover:roverrover@192.168.144.30:554/1/h264major";

    // Open the video stream

    if (RecordingMap.find(cameraURL) != RecordingMap.end())  // check if currently recording
    {
        Recording* pRecording = &RecordingMap.at(cameraURL);

        // Save the last frame from recording as picture:
        cv::imwrite(captureName, pRecording->getFrame());
        RCLCPP_INFO(LOGGER, "Screenshot saved successfully as: %s", captureName.c_str());

        // Display the frame
        cv::imshow("IP Camera Screenshot", pRecording->getFrame());

        return true;
    }
    else  // if not recording proceed normaly
    {
        std::string pipeline
            = "rtspsrc location=" + cameraURL
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

bool CameraNode::stopRecording(std::string cameraURL)
{
    std::lock_guard<std::mutex> lock(recordingMutex);

    if (RecordingMap.find(cameraURL) != RecordingMap.end())
    {
        RecordingMap.erase(cameraURL);

        if (RecordingMap.empty())
        {
            watchDogStop.store(true);
            recordingCv.notify_one();
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool CameraNode::newRecording(std::string videoFolderPath, std::string filename, std::string cameraURL)
{
    if (RecordingMap.find(cameraURL) != RecordingMap.end())  // check if recording doesn't already exist
    {
        return false;
    }
    else
    {
        RecordingMap.emplace(cameraURL,
                             Recording(videoFolderPath,
                                       filename,
                                       cameraURL,
                                       std::make_shared<rclcpp::Logger>(LOGGER),
                                       [this](std::string url)
                                       {
                                           RequestShutdown(url);
                                       }));

        if (!videoWatchDog.joinable())
        {
            StartWatchDog();
        }

        // Access the recording using at() to safely get the reference
        Recording* pRecording = &RecordingMap.at(cameraURL);

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

// gps position for file name
void CameraNode::callbackPosition(const rover_msgs::msg::GpsPosition& gps_message)
{
    last_latitude = gps_message.latitude;
    last_longitude = gps_message.longitude;
}

// Add recording key to shutdown list and notify watch dog for shutdown
void CameraNode::RequestShutdown(std::string camURL)
{
    RCLCPP_WARN(LOGGER, "Received shutdown request for %s", camURL.c_str());

    {
        std::unique_lock<std::mutex> lock(recordingMutex);
        RecordingShutdownRequestSet.insert(camURL);
    }  // unlock
    recordingCv.notify_one();
    return;
}

// start the VideoWatchDogFunction
bool CameraNode::StartWatchDog()
{
    watchDogStop.store(false);
    videoWatchDog = std::thread(&CameraNode::VideoWatchDogFunction, this);
    return true;
}

// when requested, shutdown and erase recordings that were in error from hashmap
void CameraNode::VideoWatchDogFunction()
{
    RCLCPP_DEBUG(LOGGER, "Starting video watchdog");
    while (!watchDogStop.load())
    {
        std::unique_lock<std::mutex> lock(recordingMutex);
        recordingCv.wait(lock,
                         [this]
                         {
                             return watchDogStop.load() || !RecordingShutdownRequestSet.empty();
                         });

        if (watchDogStop)
            break;
        else
        {
            for (std::string url : RecordingShutdownRequestSet)
            {
                RCLCPP_WARN(LOGGER, "Processing Shutdown for %s", url.c_str());
                if (RecordingMap.find(url) != RecordingMap.end())
                {
                    if (!RecordingMap.erase(url))
                    {
                        RCLCPP_ERROR(LOGGER, "Shutdown request for %s could not be processed, please try again", url.c_str());
                    }

                    if (RecordingMap.empty())
                    {
                        watchDogStop.store(true);
                    }
                }
                else
                {
                    RCLCPP_ERROR(LOGGER, "Unable to find %s for shutdown, please try again", url.c_str());
                }
            }

            RecordingShutdownRequestSet.clear();
        }
    }
    RCLCPP_DEBUG(LOGGER, "Stopping video watchdog");
    return;
}