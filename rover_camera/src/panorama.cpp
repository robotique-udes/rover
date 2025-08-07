#include "panorama.hpp"
#include <sys/stat.h>
#include <rover_lib2/helpers/folders.hpp>
#include <rover_lib2/helpers/date.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

Panorama::Panorama():
    Node("Panorama")
{
    _srv_panorama = this->create_service<rover_msgs::srv::Panorama>(
        PANORAMA_SERVICE_NAME,
        [this](const std::shared_ptr<rover_msgs::srv::Panorama::Request> request_,
               std::shared_ptr<rover_msgs::srv::Panorama::Response> response_)
        {
            this->handlePanoramaRequest(*request_, *response_);
        });

    _sub_gps = this->create_subscription<rover_msgs::msg::Gps>(TOPIC_GPS_NAME,
                                                               QOS_DEFAULT,
                                                               [this](const rover_msgs::msg::Gps& gpsMsg_)
                                                               {
                                                                   this->setGpsPosition(gpsMsg_);
                                                               });
    _pub_cameraCmd = this->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_CAMERA_PTZ_CMD_PANORAMA, QOS_DEFAULT);
    _pub_cameraConfig = this->create_publisher<rover_msgs::msg::CameraConfig>(TOPIC_CAMERA_CONFIG_PANORAM, QOS_DEFAULT);
}

void Panorama::handlePanoramaRequest(const rover_msgs::srv::Panorama::Request& request_,
                                     rover_msgs::srv::Panorama::Response& response_)
{
    response_.success = false;
    if (!this->validateRequest(request_, response_))
    {
        return;
    }

    std::optional<uint8_t> idCam = this->getIdCam(request_.camera_url);
    if (!idCam.has_value())
    {
        return;
    }

    this->rotateCamera(request_.duration, *idCam);

    std::vector<cv::Mat> frames;
    if (!this->captureFrames(request_, response_, frames))
    {
        _timer_ptzCmd->cancel();
        return;
    }
    this->configPtz(*idCam, 10.0F /*= As fast as possible*/);
    _timer_ptzCmd->cancel();

    std::optional<cv::Mat> pano = this->stitchFrames(frames);
    if (!pano.has_value())
    {
        RCLCPP_ERROR(this->get_logger(), "Stitching failed, panorama image is empty.");
        response_.status = "Stitching failed, panorama image is empty.";
        return;
    }

    std::optional<cv::Mat> panoRect = this->warpCorrection(*pano);
    if (!panoRect.has_value())
    {
        RCLCPP_ERROR(this->get_logger(), "Warp correction failed, panorama image is empty.");
        response_.status = "Warp correction failed, panorama image is empty.";
        return;
    }

    this->annotatePanorama(*panoRect, request_.panorama_name);

    std::string filename;
    if (!this->prepareOutputPath(request_, response_, filename))
    {
        return;
    }

    if (!this->savePanorama(response_, filename, *panoRect))
    {
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Panorama saved to %s", filename.c_str());
    response_.status = "Panorama saved to: " + filename;
    response_.success = true;
}

std::optional<cv::Mat> Panorama::warpCorrection(const cv::Mat& pano)
{
    if (pano.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Empty image was received for cropping");
        return std::nullopt;
    }

    int width = pano.cols;
    int height = pano.rows;

    int marginX = width * CROP_PERCENT;
    int marginY = height * CROP_PERCENT;

    int cropWidth = std::max(1, width - 2 * marginX);
    int cropHeight = std::max(1, height - 2 * marginY);

    if (cropWidth <= 0 || cropHeight <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid dimensions for cropping.");
        return std::nullopt;
    }

    cv::Rect roi(marginX, marginY, cropWidth, cropHeight);
    return pano(roi).clone();
}

std::optional<cv::Mat> Panorama::stitchFrames(std::vector<cv::Mat>& frames_)
{
    if (frames_.size() < 2)
    {
        RCLCPP_WARN(this->get_logger(), "Not enough images for stitching (need at least 2)");
        return std::nullopt;
    }

    cv::Mat pano;
    std::future<cv::Stitcher::Status> future = std::async(std::launch::async,
                                                          [&frames_, &pano](void)
                                                          {
                                                              cv::Ptr<cv::Stitcher> stitcher
                                                                  = cv::Stitcher::create(cv::Stitcher::PANORAMA);
                                                              cv::Stitcher::Status status = stitcher->stitch(frames_, pano);
                                                              return status;
                                                          });

    if (future.wait_for(std::chrono::milliseconds(STITCH_TIMEOUT_MS)) != std::future_status::ready)
    {
        RCLCPP_ERROR(this->get_logger(), "Stitching timed out after %d seconds", STITCH_TIMEOUT_MS);
        return std::nullopt;
    }

    if (future.get() != cv::Stitcher::OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Stitching failed. Error code: %d", static_cast<int>(future.get()));
        return std::nullopt;
    }

    return pano;
}

void Panorama::setGpsPosition(const rover_msgs::msg::Gps& gpsMsg_)
{
    _sGpsCoordinates.latitude = gpsMsg_.latitude;
    _sGpsCoordinates.longitude = gpsMsg_.longitude;
}

std::optional<std::string> Panorama::getFolderPath(const std::string& basePath_)
{
    const char* home = std::getenv("HOME");
    std::string homeStr;
    if (home)
    {
        homeStr = home;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to create session folder, $HOME env variable wasn't found");
        return std::nullopt;
    }

    std::string folderPath = homeStr + basePath_ + PATH_FOR_PANORAMA;
    return folderPath;
}

bool Panorama::validateRequest(const rover_msgs::srv::Panorama::Request& request_, rover_msgs::srv::Panorama::Response& response_)
{
    if (request_.duration <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Requested duration is zero or negative, aborting panorama.");
        response_.status = "Requested duration is zero or negative.";
        return false;
    }
    if (request_.camera_url.rfind("rtsp://", 0) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Camera URL does not start with rtsp://, aborting panorama.");
        response_.status = "Camera URL must start with rtsp:// because pipeline is rtsp specific";
        return false;
    }
    return true;
}

bool Panorama::captureFrames(const rover_msgs::srv::Panorama::Request& request_,
                             rover_msgs::srv::Panorama::Response& response_,
                             std::vector<cv::Mat>& frames_)
{
    std::string pipeline = "rtspsrc location=" + request_.camera_url + PIPELINE;
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        response_.status = "Failed to open camera stream (cap)";
        return false;
    }
    RCLCPP_DEBUG(this->get_logger(), "Starting frame capture for camera: %s", request_.camera_url.c_str());

    cv::Mat frame;
    uint8_t invalidFramesCounter = 0U;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()
           < request_.duration)
    {
        cap >> frame;
        if (!frame.empty())
        {
            frames_.push_back(frame.clone());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Blank frame grabbed");
            invalidFramesCounter++;
            if (invalidFramesCounter >= MAX_INVALID_FRAMES)
            {
                RCLCPP_ERROR(this->get_logger(), "Too many blank frame grabbed, stopping capture");
                break;
            }
        }
    }
    cap.release();
    return true;
}

void Panorama::annotatePanorama(cv::Mat& pano_, const std::string& name_)
{
    std::string gpsCoord
        = "latitude: " + std::to_string(_sGpsCoordinates.latitude) + ", longitude: " + std::to_string(_sGpsCoordinates.longitude);
    std::string text = name_.empty() ? Date::getCurrentTime() : name_ + " " + Date::getCurrentTime();

    cv::Size dimensions = pano_.size();
    int height = dimensions.height;
    cv::putText(pano_, gpsCoord, cv::Point(10, height - 20), cv::FONT_HERSHEY_SIMPLEX, FONT_SCALE, TEXT_COLOR, TEXT_THICKNESS);
    cv::putText(pano_, text, cv::Point(10, height), cv::FONT_HERSHEY_SIMPLEX, FONT_SCALE, TEXT_COLOR, TEXT_THICKNESS);
}

bool Panorama::prepareOutputPath(const rover_msgs::srv::Panorama::Request& request_,
                                 rover_msgs::srv::Panorama::Response& response_,
                                 std::string& filename_)
{
    std::optional<std::string> pathFolderOptional = this->getFolderPath(request_.base_path);
    if (!pathFolderOptional)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to find home environment when capturing panorama on camera %s",
                     request_.camera_url.c_str());
        response_.success = false;
        response_.status = "Failed to find home environment for saving screenshot on camera: " + request_.camera_url;
        return false;
    }
    std::string pathFolder = *pathFolderOptional;
    if (!Folders::createFolder(pathFolder))
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create panorama folder at %s for camera: %s",
                     pathFolder.c_str(),
                     request_.camera_url.c_str());
        response_.success = false;
        response_.status
            = "Failed to create screenshots folder or it already exists at " + pathFolder + " for camera: " + request_.camera_url;
        return false;
    }

    std::string resultName = PANORAMA_FILE_NAME + Date::getCurrentTime() + ".jpg";
    filename_ = pathFolder + "/" + resultName;
    return true;
}

bool Panorama::savePanorama(rover_msgs::srv::Panorama::Response& response_, const std::string& filename_, const cv::Mat& pano_)
{
    try
    {
        if (!cv::imwrite(filename_, pano_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save panorama image to file: %s", filename_.c_str());
            response_.success = false;
            response_.status = "Failed to save panorama image to file: " + filename_;
            return false;
        }
    }
    catch (const cv::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "OpenCV exception during imwrite: %s", e.what());
        response_.success = false;
        response_.status = std::string("OpenCV exception during imwrite: ") + e.what();
        return false;
    }
    return true;
}

void Panorama::rotateCamera(uint16_t duration_, uint8_t idCam_)
{
    float totalPanDeg = static_cast<float>(duration_) / 1000.0F * MAX_ROTATION_SPEED_PANORAMA;
    float targetRotationSpeed = MAX_ROTATION_SPEED_PANORAMA;
    if (totalPanDeg > MAX_PAN_ANGLE)
    {
        totalPanDeg = MAX_PAN_ANGLE;
        targetRotationSpeed = MAX_PAN_ANGLE / (static_cast<float>(duration_) / 1000.0F);
    }

    // Move to Start Angle
    rover_msgs::msg::CameraControl ptzMsg;
    float startAngle = degToRad(MIDDLE_PAN_ANGLE - totalPanDeg / 2.0F);
    ptzMsg.pitch = 0.0F;
    ptzMsg.power_on = true;
    ptzMsg.yaw = startAngle;
    ptzMsg.id_cam = idCam_;

    _timer_ptzCmd = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_CMD_PERIOD_MS),
                                            [this, ptzMsg](void)
                                            {
                                                _pub_cameraCmd->publish(ptzMsg);
                                            });

    _pub_cameraCmd->publish(ptzMsg);

    this->waitForAngle(idCam_, startAngle);

    this->configPtz(idCam_, degToRad(targetRotationSpeed));

    // Move to Target Angle
    ptzMsg.yaw = degToRad(MIDDLE_PAN_ANGLE + totalPanDeg / 2.0F);

    _timer_ptzCmd = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_CMD_PERIOD_MS),
                                            [this, ptzMsg](void)
                                            {
                                                _pub_cameraCmd->publish(ptzMsg);
                                            });
}

void Panorama::waitForAngle(uint8_t idCam_, float angle_)
{
    std::promise<void> angleReachedPromise;
    std::future<void> angleReachedFuture = angleReachedPromise.get_future();

    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr sub_ptzStatusTemp
        = this->create_subscription<rover_msgs::msg::CameraControl>(
            TOPIC_CAMERA_PTZ_STATUS,
            QOS_DEFAULT,
            [this, &idCam_, &angle_, &angleReachedPromise](const rover_msgs::msg::CameraControl& msg_)
            {
                if (msg_.id_cam == idCam_ && std::fabs(msg_.yaw - angle_) < POSITION_TOLERANCE)
                {
                    angleReachedPromise.set_value();
                }
            });

    if (angleReachedFuture.wait_for(std::chrono::milliseconds(ANGLE_WAIT_TIMEOUT_MS)) == std::future_status::timeout)
    {
        RCLCPP_INFO(this->get_logger(), "Desired start angle for panorama wasn't reached in time, starting panorama anyway");
    }
}

void Panorama::configPtz(uint8_t idCam_, float rotationSpeed_)
{
    rover_msgs::msg::CameraConfig configMsg;
    configMsg.tilt_max_speed = rotationSpeed_;
    configMsg.pan_max_position = degToRad(MAX_PAN_ANGLE);
    configMsg.pan_min_position = 0.0F;
    configMsg.pan_max_speed = rotationSpeed_;
    configMsg.tilt_max_position = degToRad(MAX_PAN_ANGLE);
    configMsg.tilt_min_position = 0.0F;
    configMsg.id_cam = idCam_;
    _pub_cameraConfig->publish(configMsg);
}

std::optional<uint8_t> Panorama::getIdCam(const std::string& camURL_)
{
    std::string name;
    Constants::CameraInfo::getNameFromURL(camURL_, name);
    Constants::CameraInfo::eCamNames idCam = Constants::CameraInfo::getIndexFromName(name);
    if (idCam == Constants::CameraInfo::eCamNames::MAIN || idCam == Constants::CameraInfo::eCamNames::ANTENNA)
    {
        return static_cast<uint8_t>(std::to_underlying(idCam));
    }
    return std::nullopt;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Panorama>());
    rclcpp::shutdown();
    return 0;
}
