#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/photo_panoramique.hpp>
#include <rover_msgs/srv/photo_panoramique.hpp>
#include <rover_msgs/msg/gps_position.hpp>
#include <iostream>
#include <vector>
#include <sys/stat.h>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

// cam: 192.168.144.30

struct emplacement
{
    float latitude;
    float longitude;
};

class PhotoPanoramique : public rclcpp::Node
{
  public:
    PhotoPanoramique();

  private:
    // attribut pour gérer les coordonées gps
    emplacement coordonees_gps;

    // fonction pour enlever le warping
    cv::Mat warp_correction(cv::Mat pano)
    {
        cv::Size dimensions = pano.size();

        int width = dimensions.width;
        int hauteur = dimensions.height;

        cv::Rect coupe(300, 300, width - 500, hauteur - 500);
        cv::Mat panoRectangle = pano(coupe);

        return panoRectangle;
    }

    // fonction pour le stitching de la photo
    cv::Mat stitching(std::vector<cv::Mat> images_cam)
    {
        cv::Mat pano;
        RCLCPP_INFO(this->get_logger(), "Maintenant en essai de stitching");
        cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);
        stitcher->stitch(images_cam, pano);

        return pano;
    }

    // fonction pour aller chercher la position GPS
    void PositionGPS(const rover_msgs::msg::GpsPosition& gpsMessage_)
    {
        coordonees_gps.latitude = gpsMessage_.latitude;
        coordonees_gps.longitude = gpsMessage_.longitude;
    }

    // section necessitees ROS
    rclcpp::Publisher<rover_msgs::msg::PhotoPanoramique>::SharedPtr _pubpanorama;
    rclcpp::TimerBase::SharedPtr _timerPub;
    rclcpp::Service<rover_msgs::srv::PhotoPanoramique>::SharedPtr _srvpanorama;
    rclcpp::Subscription<rover_msgs::msg::GpsPosition>::SharedPtr _sub_position;

    rover_msgs::msg::PhotoPanoramique _msgPanorama;
    void CB_timer(void);
    void sendCmd(void);
    void CB_srv(const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request,
                std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response);
};

PhotoPanoramique::PhotoPanoramique():
    Node("photo_panoramique")
{
    _pubpanorama = this->create_publisher<rover_msgs::msg::PhotoPanoramique>("/rover/video/panorama", QOS_DEFAULT);

    _srvpanorama = this->create_service<rover_msgs::srv::PhotoPanoramique>(
        "/rover/video/panorama",
        [this](const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request_,
               std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response_)
        {
            this->CB_srv(request_, response_);
        });

    _sub_position
        = this->create_subscription<rover_msgs::msg::GpsPosition>("/rover/gps/position",
                                                                  QOS_DEFAULT,
                                                                  [this](const rover_msgs::msg::GpsPosition& gps_message_)
                                                                  {
                                                                      this->PositionGPS(gps_message_);
                                                                  });
}

void PhotoPanoramique::CB_srv(const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request,
                              std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response)
{
    response->success = false;

    RCLCPP_INFO(this->get_logger(), "Incoming request");

    if (request->start == true)
    {
        RCLCPP_INFO(this->get_logger(), "Panorama started");

        // paramètres pour le stitching
        int num = request->pano_number;
        std::string numero = std::to_string(num);
        std::string result_name = "panorama_" + numero;
        result_name = result_name + ".jpg";

        // paramètres pour la lecture de la camera

        // Pour debugger via camera usb
        // std::string path_camera= request->camera_id;
        // cv::VideoCapture cap;
        // int apiID = cv::CAP_ANY;
        // cap.open(path_camera, apiID);

        std::string pipeline = "rtspsrc location= rtsp://rovus:rovusrovus@" + request->camera_id
                               + ":554/1/h264major latency=0 drop=true ! decodebin ! videorate max-rate=30 ! videoconvert ! "
                                 "queue max-size-buffers=1 ! appsink";

        // Connection a la camera
        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
        int apiID = cv::CAP_GSTREAMER;

        if (cap.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "Camera open");
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to open camera");
        }

        // paramètres pour le traitement des images
        cv::Mat frame;
        std::vector<cv::Mat> images_cam;
        bool taking_panorama = true;

        RCLCPP_INFO(this->get_logger(), "Début de la capture vidéo pour la panoramique");

        // création de la liste d'image
        int i = 0;
        while (taking_panorama)
        {
            cap.read(frame);

            // Stocker les images pour le panorama
            if (i % 5 == 0)  // ici pour changer la fréquence de prise d'images
            {
                images_cam.push_back(frame.clone());
            }

            i = i + 1;

            // Gestion du temps alloue pour prendre la panoramique
            if (i == 200)  // ici pour changer la quantite de frames a attendre avant d'arreter
            {
                taking_panorama = false;
                RCLCPP_INFO(this->get_logger(), "Arrêté avec succès");
            }
        }

        // Fermer la camera après la capture
        cap.release();

        // stitching de la panoramique
        cv::Mat pano = stitching(images_cam);

        // correction du warping
        cv::Mat pano_rectangle = warp_correction(pano);

        // obtenir coordonees GPS
        float latitude = coordonees_gps.latitude;
        float longitude = coordonees_gps.longitude;
        std::string coord_GPS = "latitude: " + std::to_string(latitude) + ", longitude: " + std::to_string(longitude);

        // ajout du text
        cv::Size dimensions = pano_rectangle.size();
        int hauteur = dimensions.height;
        std::string nom_photo = request->nom;
        putText(pano_rectangle,
                coord_GPS,
                cv::Point(10, hauteur - 20),
                cv::FONT_HERSHEY_SIMPLEX,
                3.0,
                cv::Scalar(34, 139, 34),
                5);
        putText(pano_rectangle,
                nom_photo,
                cv::Point(10, hauteur - 120),
                cv::FONT_HERSHEY_SIMPLEX,
                3.0,
                cv::Scalar(34, 139, 34),
                5);

        // creation du dossier du dossier de panoramas
        std::string currentPackageDirectory = GET_PACKAGE_SOURCE_DIR("rover_camera");
        std::string path_panorama = "/src/panoramas";

        struct stat fileInfo;
        std::string nom_fichier_panorama;
        std::string path_dossier = currentPackageDirectory + path_panorama;
        bool dossier_exist = stat(path_dossier.c_str(), &fileInfo) == 0;

        if (!dossier_exist)  // creation du dossier si necessaire
        {
            RCLCPP_INFO(this->get_logger(), "The folder doesn't exist yet");

            if (mkdir(path_dossier.c_str(), 0775) == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Succesfully created the folder");
                nom_fichier_panorama = path_dossier + "/" + result_name;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to create folder");
                nom_fichier_panorama = result_name;
                imwrite(result_name, pano_rectangle);  // enregistre quand meme mais potentielement hors folder
            }
        }
        else
        {
            nom_fichier_panorama = path_dossier + "/" + result_name;
        }

        // enregistrement de la panoramique
        imwrite(nom_fichier_panorama, pano_rectangle);

        RCLCPP_INFO(this->get_logger(), "Panorama done");

        response->success = true;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PhotoPanoramique>());
    rclcpp::shutdown();
    return 0;
}
