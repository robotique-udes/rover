#include "panorama.hpp"
#include <rover_lib2/helpers/folders.hpp>

// cam: 192.168.144.30

PhotoPanoramique::PhotoPanoramique():
    Node("photo_panoramique")
{
    srv_panorama = this->create_service<rover_msgs::srv::PhotoPanoramique>(
        PANORAMA_SERVICE_NAME,
        [this](const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request_,
               std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response_)
        {
            this->CB_srv(request_, response_);
        });

    sub_position
        = this->create_subscription<rover_msgs::msg::Gps>(TOPIC_GPS_NAME,
                                                                  QOS_DEFAULT,
                                                                  [this](const rover_msgs::msg::Gps& gps_message_)
                                                                  {
                                                                      this->PositionGPS(gps_message_);
                                                                  });
}

void PhotoPanoramique::CB_srv(const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request_,
                              std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response_)
{
    response_->success = false;

    // paramètres pour le stitching
    std::string resultName = "panorama_" + getCurrentTime() + ".jpg";

    // paramètres pour la lecture de la camera

    // Pour debugger via camera usb
    // int apiID = cv::CAP_GSTREAMER;
    // std::string path_camera= request->camera_id;
    // cv::VideoCapture cap;
    // int apiID = cv::CAP_ANY;
    // cap.open(path_camera, apiID);

    std::string pipeline = "rtspsrc location= rtsp://rovus:rovusrovus@" + request_->camera_url
                           + ":554/1/h264major latency=0 drop=true ! decodebin ! videorate max-rate=30 ! videoconvert ! "
                             "queue max-size-buffers=1 ! appsink";

    // Connection a la camera
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to open camera");
        response_->status = "Failed to open camera stream (cap)";
        return;
    }

    // paramètres pour le traitement des images
    cv::Mat frame;
    std::vector<cv::Mat> imagesCam;

    RCLCPP_INFO(this->get_logger(), "Starting panorama");

    // création de la liste d'image

    bool frameValid;
    uint8_t invalidFramesCounter = 0;
    for (size_t i = 0; i < FPS * request_->duration / 1000; i++)
    {
        frameValid = cap.read(frame);

        if (!frameValid)
        {
            RCLCPP_ERROR(this->get_logger(), "Blank frame grabbed");
            invalidFramesCounter++;
        }

        // Stocker les images pour le panorama
        if (i % FRAMES_TO_SKIP == 0 && frameValid)  // ici pour changer la fréquence de prise d'images
        {
            imagesCam.push_back(frame.clone());
        }

        if (invalidFramesCounter >= MAX_INVALID_FRAMES)
        {
            RCLCPP_ERROR(this->get_logger(), "Too many blank frame grabbed, stopping capture");
            break;
        }
    }

    // Fermer la camera après la capture
    cap.release();

    // stitching de la panoramique
    cv::Mat pano = stitching(imagesCam);

    // correction du warping
    cv::Mat panoRectangle = warpCorrection(pano);

    // obtenir coordonees GPS
    float latitude = sCoordoneesGps.latitude;
    float longitude = sCoordoneesGps.longitude;
    std::string coordGps = "latitude: " + std::to_string(latitude) + ", longitude: " + std::to_string(longitude);

    // ajout du text
    cv::Size dimensions = panoRectangle.size();
    int hauteur = dimensions.height;
    std::string nomPhoto = request_->panorama_name;
    putText(panoRectangle, coordGps, cv::Point(10, hauteur - 20), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(34, 139, 34), 5);
    putText(panoRectangle, nomPhoto, cv::Point(10, hauteur - 120), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(34, 139, 34), 5);

    // creation du dossier du dossier de panoramas
    std::optional<std::string> pathFolderOptional = this->getFolderPath(request_->base_path);
    if (!pathFolderOptional)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to find home environment when capturing panorama on camera %s",
                     request_->camera_url.c_str());
        response_->success = false;
        response_->status = "Failed to find home environment for saving screenshot on camera: " + request_->camera_url;
        return;
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
    }

    std::string filename = pathFolder + "/" + resultName;

    // enregistrement de la panoramique
    imwrite(filename, panoRectangle);

    RCLCPP_INFO(this->get_logger(), "Panorama done");

    response_->success = true;
}

cv::Mat PhotoPanoramique::warpCorrection(const cv::Mat& pano)
{
    if (pano.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Empty image was received for croping");
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

cv::Mat PhotoPanoramique::stitching(std::vector<cv::Mat>& imagesCam)
{
    if (imagesCam.empty())
    {
        RCLCPP_WARN(this->get_logger(), "La liste d'images est vide. Abandon du stitching.");
        return cv::Mat();  // retourne une image vide
    }

    cv::Mat pano;
    RCLCPP_DEBUG(this->get_logger(), "Maintenant en essai de stitching");
    cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);

    cv::Stitcher::Status status = stitcher->stitch(imagesCam, pano);

    if (status != cv::Stitcher::OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Échec du stitching. Code erreur : %d", static_cast<int>(status));
        return cv::Mat();  // retourne une image vide en cas d'échec
    }

    return pano;
}

void PhotoPanoramique::PositionGPS(const rover_msgs::msg::Gps& gpsMessage_)
{
    sCoordoneesGps.latitude = gpsMessage_.latitude;
    sCoordoneesGps.longitude = gpsMessage_.longitude;
}

std::string PhotoPanoramique::getCurrentTime(void)
{
    std::stringstream current_time_output;

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);  // convert to real time
    std::tm tm_now = *std::localtime(&now_time);                       // convert to calendar time
    current_time_output << std::put_time(&tm_now, "%FT%T");            // ISO 8601 format

    return current_time_output.str();
}

std::optional<std::string> PhotoPanoramique::getFolderPath(const std::string& basePath_)
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
        RCLCPP_ERROR(this->get_logger(), "Unable to create session folder, $HOME env variable wasn't found");
    }

    std::string folderPath = homeStr + basePath_ + PATH_FOR_PANORAMA;
    return folderPath;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PhotoPanoramique>());
    rclcpp::shutdown();
    return 0;
}
