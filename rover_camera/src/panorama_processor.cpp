#include "panorama_processor.hpp"

#include <sys/stat.h>
#include <rover_lib2/helpers/folders.hpp>
#include <rover_lib2/helpers/date.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_lib2/helpers/chrono.hpp>
#include <rover_lib2/helpers/time.hpp>

const cv::Scalar PanoramaProcessor::TEXT_COLOR = cv::Scalar(34, 139, 34);
const rclcpp::Logger PanoramaProcessor::LOGGER = rclcpp::get_logger("PanoramaManager");

PanoramaProcessor::PanoramaProcessor(std::weak_ptr<rclcpp::Node> node_,
                                     Constants::CameraInfo::eCamNames id_,
                                     std::shared_ptr<CameraInterface> cameraInterface_):
    _node(node_),
    _id(id_),
    _cameraInterface(cameraInterface_)
{
}

PanoramaProcessor::~PanoramaProcessor()
{
    if (this->isBusy())
    {
        std::unique_lock dummyLock(_dummyMutex);
        if (!_panoramaDone.wait_for(dummyLock,
                                    SHUTDOWN_LIMIT_MS,
                                    [this](void)
                                    {
                                        return !this->isBusy();
                                    }))
        {
            RCLCPP_WARN(LOGGER, "Shutdown limit reached, forcing shutdown");
        }
    }
}

void PanoramaProcessor::execute(const rover_msgs::srv::Panorama::Request& request_,
                                rover_msgs::srv::Panorama::Response& response_,
                                Constants::CameraInfo::eCamNames id_,
                                sCoordinate coordinates_)
{
    if (this->isBusy())
    {
        response_.success = false;
        response_.status = "Another panorama is already in progress for this camera";
        return;
    }

    _busy.store(true);

    this->enableCameraPower(id_);
    this->handlePanoramaRequest(request_, response_, id_, coordinates_);
    this->disableCameraPower(id_);

    if (_cameraInterface)
    {
        _cameraInterface->release(id_);
    }

    _busy.store(false);
    _panoramaDone.notify_one();
}

bool PanoramaProcessor::isBusy(void)
{
    return _busy.load();
}

void PanoramaProcessor::handlePanoramaRequest(const rover_msgs::srv::Panorama::Request& request_,
                                              rover_msgs::srv::Panorama::Response& response_,
                                              Constants::CameraInfo::eCamNames id_,
                                              sCoordinate coordinates_)
{
    response_.success = false;
    if (!this->validateRequest(request_, response_))
    {
        return;
    }
    std::chrono::milliseconds duration = std::chrono::milliseconds(request_.duration);
    this->rotateCamera(duration, id_);

    std::vector<cv::Mat> frames;
    if (!this->captureFrames(request_, response_, frames))
    {
        return;
    }

    std::optional<cv::Mat> pano = this->stitchFrames(frames, response_);
    if (!pano)
    {
        return;
    }

    std::optional<cv::Mat> panoRect = this->warpCorrection(*pano, response_);
    if (!panoRect)
    {
        return;
    }

    this->annotatePanorama(*panoRect, request_.panorama_name, coordinates_);

    std::string filename;
    if (!this->prepareOutputPath(request_, response_, filename))
    {
        return;
    }

    if (!this->savePanorama(response_, filename, *panoRect))
    {
        return;
    }

    RCLCPP_DEBUG(LOGGER, "Panorama saved to %s", filename.c_str());
    response_.status = "Panorama saved to: " + filename;
    response_.success = true;
}

std::optional<cv::Mat> PanoramaProcessor::warpCorrection(const cv::Mat& pano, rover_msgs::srv::Panorama::Response& response_)
{
    if (pano.empty())
    {
        RCLCPP_ERROR(LOGGER, "Empty image was received for cropping");
        response_.status = "Warp correction failed, panorama image is empty.";
        return std::nullopt;
    }

    const int width = pano.cols;
    const int height = pano.rows;

    const int marginX = width * CROP_PERCENT;
    const int marginY = height * CROP_PERCENT;

    const int cropWidth = std::max(1, width - 2 * marginX);
    const int cropHeight = std::max(1, height - 2 * marginY);

    if (cropWidth <= 0 || cropHeight <= 0)
    {
        RCLCPP_ERROR(LOGGER, "Invalid dimensions for cropping.");
        response_.status = "Warp correction failed, panorama image dimensions are invalid.";
        return std::nullopt;
    }

    cv::Rect roi(marginX, marginY, cropWidth, cropHeight);
    return pano(roi).clone();
}

std::optional<cv::Mat> PanoramaProcessor::stitchFrames(std::vector<cv::Mat>& frames_,
                                                       rover_msgs::srv::Panorama::Response& response_)
{
    if (frames_.size() < 2)
    {
        RCLCPP_WARN(LOGGER, "Not enough images for stitching (need at least 2)");
        response_.status = "Stitching failed, not enough images were captured (min 2).";
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

    if (future.wait_for(STITCH_TIMEOUT_MS) != std::future_status::ready)
    {
        RCLCPP_ERROR(LOGGER, "Stitching timed out after %ld seconds", STITCH_TIMEOUT_MS.count());
        response_.status = "Stitching timeout after" + std::to_string(STITCH_TIMEOUT_MS.count()) + " ms";
        return std::nullopt;
    }

    if (future.get() != cv::Stitcher::OK)
    {
        RCLCPP_ERROR(LOGGER, "Stitching failed. Error code: %d", static_cast<int>(future.get()));
        response_.status = "Stitching failed unexpectedly, check logs for reason";
        return std::nullopt;
    }

    return pano;
}

std::optional<std::string> PanoramaProcessor::getFolderPath(const std::string& basePath_)
{
    std::optional<std::string> home = Folders::getHome();
    if (!home)
    {
        return std::nullopt;
    }

    std::string folderPath = *home + basePath_ + PATH_FOR_PANORAMA;
    return folderPath;
}

bool PanoramaProcessor::validateRequest(const rover_msgs::srv::Panorama::Request& request_,
                                        rover_msgs::srv::Panorama::Response& response_)
{
    if (request_.duration <= 0)
    {
        RCLCPP_ERROR(LOGGER, "Requested duration is zero or negative, aborting panorama.");
        response_.status = "Requested duration is zero or negative.";
        return false;
    }
    if (request_.camera_url.rfind("rtsp://", 0) != 0)
    {
        RCLCPP_ERROR(LOGGER, "Camera URL does not start with rtsp://, aborting panorama.");
        response_.status = "Camera URL must start with rtsp:// because pipeline is rtsp specific";
        return false;
    }
    return true;
}

bool PanoramaProcessor::captureFrames(const rover_msgs::srv::Panorama::Request& request_,
                                      rover_msgs::srv::Panorama::Response& response_,
                                      std::vector<cv::Mat>& frames_)
{
    std::string pipeline = "rtspsrc location=" + request_.camera_url + PIPELINE;
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        RCLCPP_ERROR(LOGGER, "Failed to open camera");
        response_.status = "Failed to open camera stream (cap)";
        return false;
    }
    RCLCPP_DEBUG(LOGGER, "Starting frame capture for camera: %s", request_.camera_url.c_str());

    cv::Mat frame;
    uint8_t invalidFramesCounter = 0U;
    Chrono<uint64_t, Time::millis> chrono;
    while (chrono.getTime() < static_cast<uint64_t>(request_.duration))
    {
        cap >> frame;
        if (!frame.empty())
        {
            frames_.push_back(frame.clone());
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Blank frame grabbed");
            invalidFramesCounter++;
            if (invalidFramesCounter >= MAX_INVALID_FRAMES)
            {
                RCLCPP_ERROR(LOGGER, "Too many blank frame grabbed, stopping capture");
                break;
            }
        }
    }
    cap.release();
    return true;
}

void PanoramaProcessor::annotatePanorama(cv::Mat& pano_, const std::string& name_, sCoordinate coordinates_)
{
    std::string gpsCoord
        = "latitude: " + std::to_string(coordinates_.latitude) + ", longitude: " + std::to_string(coordinates_.longitude);
    std::string text = name_.empty() ? Date::getCurrentTime() : name_ + " " + Date::getCurrentTime();

    cv::Size dimensions = pano_.size();
    int height = dimensions.height;
    cv::putText(pano_, gpsCoord, cv::Point(10, height - 20), cv::FONT_HERSHEY_SIMPLEX, FONT_SCALE, TEXT_COLOR, TEXT_THICKNESS);
    cv::putText(pano_, text, cv::Point(10, height), cv::FONT_HERSHEY_SIMPLEX, FONT_SCALE, TEXT_COLOR, TEXT_THICKNESS);
}

bool PanoramaProcessor::prepareOutputPath(const rover_msgs::srv::Panorama::Request& request_,
                                          rover_msgs::srv::Panorama::Response& response_,
                                          std::string& filename_)
{
    std::optional<std::string> pathFolderOptional = this->getFolderPath(request_.base_path);
    if (!pathFolderOptional)
    {
        RCLCPP_ERROR(LOGGER, "Failed to find home environment when capturing panorama on camera %s", request_.camera_url.c_str());
        response_.success = false;
        response_.status = "Failed to find home environment for saving screenshot on camera: " + request_.camera_url;
        return false;
    }
    std::string pathFolder = *pathFolderOptional;
    if (!Folders::createFolder(pathFolder))
    {
        RCLCPP_ERROR(LOGGER,
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

bool PanoramaProcessor::savePanorama(rover_msgs::srv::Panorama::Response& response_,
                                     const std::string& filename_,
                                     const cv::Mat& pano_)
{
    try
    {
        if (!cv::imwrite(filename_, pano_))
        {
            RCLCPP_ERROR(LOGGER, "Failed to save panorama image to file: %s", filename_.c_str());
            response_.success = false;
            response_.status = "Failed to save panorama image to file: " + filename_;
            return false;
        }
    }
    catch (const cv::Exception& e)
    {
        RCLCPP_ERROR(LOGGER, "OpenCV exception during imwrite: %s", e.what());
        response_.success = false;
        response_.status = std::string("OpenCV exception during imwrite: ") + e.what();
        return false;
    }
    return true;
}

void PanoramaProcessor::rotateCamera(std::chrono::milliseconds duration_, Constants::CameraInfo::eCamNames id_)
{
    float totalPanDeg = static_cast<float>(duration_.count()) / 1000.0F * MAX_ROTATION_SPEED_PANORAMA;
    float targetRotationSpeed = MAX_ROTATION_SPEED_PANORAMA;
    if (totalPanDeg > MAX_PAN_ANGLE)
    {
        totalPanDeg = MAX_PAN_ANGLE;
        targetRotationSpeed = MAX_PAN_ANGLE / (static_cast<float>(duration_.count()) / 1000.0F);
    }

    // Move to Start Angle
    rover_msgs::msg::CameraControl ptzMsg;
    float startAngle = degToRad(MIDDLE_PAN_ANGLE - totalPanDeg / 2.0F);
    ptzMsg.pitch = 0.0F;
    ptzMsg.power_on = true;
    ptzMsg.yaw = startAngle;
    ptzMsg.id_cam = std::to_underlying(id_);

    if (_cameraInterface)
    {
        _cameraInterface->setPTZCmd(ptzMsg, id_);
        this->waitForAngle(id_, startAngle);
    }

    this->configPtz(id_, degToRad(targetRotationSpeed));

    // Move to Target Angle
    ptzMsg.yaw = degToRad(MIDDLE_PAN_ANGLE + totalPanDeg / 2.0F);
    if (_cameraInterface)
    {
        _cameraInterface->setPTZCmd(ptzMsg, id_);
    }
}

void PanoramaProcessor::waitForAngle(Constants::CameraInfo::eCamNames id_, float angle_)
{
    std::promise<void> angleReachedPromise;
    std::future<void> angleReachedFuture = angleReachedPromise.get_future();

    if (std::shared_ptr<rclcpp::Node> lockedNode = _node.lock())
    {
        rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr sub_ptzStatusTemp
            = lockedNode->create_subscription<rover_msgs::msg::CameraControl>(
                TOPIC_CAMERA_PTZ_STATUS,
                QOS_DEFAULT,
                [this, &id_, &angle_, &angleReachedPromise](const rover_msgs::msg::CameraControl& msg_)
                {
                    if (msg_.id_cam == std::to_underlying(id_) && std::fabs(msg_.yaw - angle_) < POSITION_TOLERANCE)
                    {
                        angleReachedPromise.set_value();
                    }
                });

        if (angleReachedFuture.wait_for(ANGLE_WAIT_TIMEOUT_MS) == std::future_status::timeout)
        {
            RCLCPP_INFO(LOGGER, "Desired start angle for panorama wasn't reached in time, starting panorama anyway");
        }
    }
}

void PanoramaProcessor::configPtz(Constants::CameraInfo::eCamNames id_, float rotationSpeed_)
{
    rover_msgs::msg::CameraConfig configMsg;
    configMsg.tilt_max_speed = rotationSpeed_;
    configMsg.pan_max_position = degToRad(MAX_PAN_ANGLE);
    configMsg.pan_min_position = 0.0F;
    configMsg.pan_max_speed = rotationSpeed_;
    configMsg.tilt_max_position = degToRad(MAX_PAN_ANGLE);
    configMsg.tilt_min_position = 0.0F;
    configMsg.id_cam = std::to_underlying(id_);

    if (_cameraInterface)
    {
        _cameraInterface->setPTZConfig(configMsg, id_);
    }
}

void PanoramaProcessor::enableCameraPower(Constants::CameraInfo::eCamNames id_)
{
    rover_msgs::msg::CameraControl msg;
    msg.id_cam = std::to_underlying(id_);
    msg.power_on = true;

    if (_cameraInterface)
    {
        _cameraInterface->setPowerCmd(msg, id_);
    }
}

void PanoramaProcessor::disableCameraPower(Constants::CameraInfo::eCamNames id_)
{
    rover_msgs::msg::CameraControl msg;
    msg.id_cam = std::to_underlying(id_);
    msg.power_on = false;
    if (_cameraInterface)
    {
        _cameraInterface->setPowerCmd(msg, id_);
    }
}