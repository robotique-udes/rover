#include "panorama.hpp"
#include <rover_lib2/helpers/folders.hpp>
#include <rover_lib2/helpers/date.hpp>

Panorama::Panorama():
    Node("photo_panoramique")
{
    srv_panorama = this->create_service<rover_msgs::srv::PhotoPanoramique>(
        PANORAMA_SERVICE_NAME,
        [this](const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request_,
               std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response_)
        {
            this->handlePanoramaRequest(request_, response_);
        });

    sub_gps = this->create_subscription<rover_msgs::msg::Gps>(TOPIC_GPS_NAME,
                                                              QOS_DEFAULT,
                                                              [this](const rover_msgs::msg::Gps& gpsMsg_)
                                                              {
                                                                  this->SetGpsPosition(gpsMsg_);
                                                              });
}

void Panorama::handlePanoramaRequest(const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request_,
                                     std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response_)
{
    response_->success = false;
    if (!validateRequest(request_, response_))
    {
        return;
    }

    std::vector<cv::Mat> frames;
    if (!captureFrames(request_, response_, frames))
    {
        return;
    }

    cv::Mat pano = this->stitching(frames);
    if (pano.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Stitching failed, panorama image is empty.");
        response_->success = false;
        response_->status = "Stitching failed, panorama image is empty.";
        return;
    }

    cv::Mat panoRect = this->warpCorrection(pano);
    if (panoRect.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Warp correction failed, panorama image is empty.");
        response_->success = false;
        response_->status = "Warp correction failed, panorama image is empty.";
        return;
    }

    this->annotatePanorama(panoRect, request_->panorama_name);

    std::string filename;
    if (!this->prepareOutputPath(request_, response_, filename))
    {
        return;
    }

    if (!this->savePanorama(response_, filename, panoRect))
    {
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Panorama saved to %s", filename.c_str());
    response_->status = "Panorama saved to: " + filename;
    response_->success = true;
}

cv::Mat Panorama::warpCorrection(const cv::Mat& pano)
{
    if (pano.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Empty image was received for croping");
        return pano;
    }

    int width = pano.cols;
    int height = pano.rows;

    int marginX = width * CROP_PERCENT;
    int marginY = height * CROP_PERCENT;

    int cropWidth = std::max(1, width - 2 * marginX);
    int cropHeight = std::max(1, height - 2 * marginY);

    if (cropWidth <= 0 || cropHeight <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid dimensions for croping.");
        return pano;
    }

    cv::Rect roi(marginX, marginY, cropWidth, cropHeight);
    return pano(roi).clone();
}

cv::Mat Panorama::stitching(std::vector<cv::Mat>& frames_)
{
    if (frames_.size() < 2)
    {
        RCLCPP_WARN(this->get_logger(), "Not enough images for stitching (need at least 2)");
        return cv::Mat();
    }

    cv::Mat pano;
    RCLCPP_DEBUG(this->get_logger(), "Maintenant en essai de stitching");
    cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);

    cv::Stitcher::Status status = stitcher->stitch(frames_, pano);

    if (status != cv::Stitcher::OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Échec du stitching. Code erreur : %d", static_cast<int>(status));
        return cv::Mat();  // retourne une image vide en cas d'échec
    }

    return pano;
}

void Panorama::SetGpsPosition(const rover_msgs::msg::Gps& gpsMessage_)
{
    _sCoordoneesGps.latitude = gpsMessage_.latitude;
    _sCoordoneesGps.longitude = gpsMessage_.longitude;
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

bool Panorama::validateRequest(const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request_,
                               std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response_)
{
    if (request_->duration <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Requested duration is zero or negative, aborting panorama.");
        response_->status = "Requested duration is zero or negative.";
        return false;
    }
    return true;
}

bool Panorama::captureFrames(const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request_,
                             std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response_,
                             std::vector<cv::Mat>& frames_)
{
    std::string pipeline = "rtspsrc location=" + request_->camera_url + PIPELINE;
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        response_->status = "Failed to open camera stream (cap)";
        return false;
    }
    RCLCPP_DEBUG(this->get_logger(), "Starting panorama for camera: %s", request_->camera_url.c_str());

    cv::Mat frame;
    uint8_t invalidFramesCounter = 0U;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()
           < request_->duration)
    {
        cap >> frame;
        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Blank frame grabbed");
            invalidFramesCounter++;
            continue;
        }
        else if (frames_.size() == 0 || (frames_.back().data != frame.data))  // avoid duplicates
        {
            frames_.push_back(frame.clone());
        }

        if (invalidFramesCounter >= MAX_INVALID_FRAMES)
        {
            RCLCPP_ERROR(this->get_logger(), "Too many blank frame grabbed, stopping capture");
            break;
        }
    }
    cap.release();
    return true;
}

void Panorama::annotatePanorama(cv::Mat& pano, const std::string& name_)
{
    float latitude = _sCoordoneesGps.latitude;
    float longitude = _sCoordoneesGps.longitude;
    std::string gpsCoord = "latitude: " + std::to_string(latitude) + ", longitude: " + std::to_string(longitude);
    std::string text = name_.empty() ? Date::getCurrentTime() : name_ + " " + Date::getCurrentTime();

    cv::Size dimensions = pano.size();
    int height = dimensions.height;
    cv::putText(pano, gpsCoord, cv::Point(10, height - 20), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(34, 139, 34), 3);
    cv::putText(pano, text, cv::Point(10, height), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(34, 139, 34), 3);
}

bool Panorama::prepareOutputPath(const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request_,
                                 std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response_,
                                 std::string& filename_)
{
    std::optional<std::string> pathFolderOptional = this->getFolderPath(request_->base_path);
    if (!pathFolderOptional)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to find home environment when capturing panorama on camera %s",
                     request_->camera_url.c_str());
        response_->success = false;
        response_->status = "Failed to find home environment for saving screenshot on camera: " + request_->camera_url;
        return false;
    }
    std::string pathFolder = pathFolderOptional.value();
    if (!Folders::createFolder(pathFolder))
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create panorama folder at %s for camera: %s",
                     pathFolder.c_str(),
                     request_->camera_url.c_str());
        response_->success = false;
        response_->status = "Failed to create screenshots folder or it already exists at " + pathFolder
                            + " for camera: " + request_->camera_url;
        return false;
    }

    std::string resultName = "panorama_" + Date::getCurrentTime() + ".jpg";
    filename_ = pathFolder + "/" + resultName;
    return true;
}

bool Panorama::savePanorama(std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response_,
                            const std::string& filename_,
                            const cv::Mat& pano_)
{
    try
    {
        if (!cv::imwrite(filename_, pano_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save panorama image to file: %s", filename_.c_str());
            response_->success = false;
            response_->status = "Failed to save panorama image to file: " + filename_;
            return false;
        }
    }
    catch (const cv::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "OpenCV exception during imwrite: %s", e.what());
        response_->success = false;
        response_->status = std::string("OpenCV exception during imwrite: ") + e.what();
        return false;
    }
    return true;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Panorama>());
    rclcpp::shutdown();
    return 0;
}
