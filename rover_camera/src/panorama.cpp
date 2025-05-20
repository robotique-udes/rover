#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/photo_panoramique.hpp"
#include "rover_msgs/srv/photo_panoramique.hpp"
#include "rover_msgs/msg/gps_position.hpp"
#include <iostream>
#include <vector>
#include <sys/stat.h>

//cam: 192.168.144.30

using namespace std;
using namespace cv;

struct {             
	float latitude;
	float longitude; 
} coordonees_gps; 


class PhotoPanoramique : public rclcpp::Node
{
  public:
    PhotoPanoramique();
    
   private:
   //fonction pour enlever le warping
    Mat warp_correction(Mat pano) {

	    Size dimensions = pano.size();
	    
	    int width = dimensions.width;
	    int hauteur = dimensions.height;
	    
	    Rect coupe(50, 50, width-100, hauteur-100);
	    Mat pano_rectangle = pano(coupe);
	    
	    return pano_rectangle;
	}
	
     //fonction pour le stitching de la photo
     Mat stitching(vector<Mat> images_cam)  {
	    Mat pano;
	    cout<<"Maintenant en essai de stitching"<<endl;
	    Ptr<Stitcher> stitcher = Stitcher::create(Stitcher::PANORAMA);
	    stitcher->stitch(images_cam, pano);
	 
	     return pano;
	}
	
    //fonction pour aller chercher la position GPS
    void PositionGPS(const rover_msgs::msg::GpsPosition& gps_message_)
	    {
	    
	    coordonees_gps.latitude = gps_message_.latitude;
	    coordonees_gps.longitude = gps_message_.longitude;	    
	    }
	    
    //section necessitees ROS
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
    _pubpanorama = this->create_publisher<rover_msgs::msg::PhotoPanoramique>("/rover/video/panorama", 1);
    
    _srvpanorama = this->create_service<rover_msgs::srv::PhotoPanoramique>(
        "/rover/video/panorama",
        std::bind(&PhotoPanoramique::CB_srv, this, std::placeholders::_1, std::placeholders::_2));
        

    _sub_position = this->create_subscription<rover_msgs::msg::GpsPosition>("/rover/gps/position", 1, [this](const rover_msgs::msg::GpsPosition& gps_message_)
                                                                  {
                                                                      this->PositionGPS(gps_message_);
                                                                  });
        
}


void PhotoPanoramique::CB_srv(const std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Request> request,
                               std::shared_ptr<rover_msgs::srv::PhotoPanoramique::Response> response)
{
    response->success = false;
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
    
   if(request->start == true){
          cout<<"Panorama started"<<endl;


	  //paramètres pour le stitching
	  int num = request->pano_number;
	  string numero = to_string(num);
	  string result_name = "panorama"+numero;
	  result_name =result_name+".jpg";
	  
	  //paramètres pour la lecture de la camera
	   string path_camera= request->camera_id;
	   //string pipeline = "rtspsrc location=‘rtsp://" + request->camera_id + "’/1/h264major’ latency=0 ! decodebin ! videoconvert ! autovideosink sync=false";
	   //string pipeline = "rtspsrc location=rtsp://192.168.144.30:554/1/h264major latency=0 ! decodebin ! videoconvert ! autovideosink sync=false";
	   
	   //VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
	   VideoCapture cap;
	   //int apiID = cv::CAP_GSTREAMER;
	   int apiID = cv::CAP_ANY;
	   cap.open(path_camera, apiID);
	   
	   if(cap.isOpened()){
	    cout << "camera open" << endl;
	    }else{
	    cout << "failed to open camera" << endl;
	    }


	   //paramètres pour le traitement des images
	    Mat frame;
	    vector<Mat> images_cam;
	    bool taking_panorama = true;

   	    cout << "Début de la capture vidéo pour la panoramique" << endl;
   
	   //création de la liste d'image
	    int i=0;
	    while (taking_panorama) 
	    {
		cap.read(frame);

		 // Stocker les images pour le panorama
		if (i % 5 == 0)
		{
		images_cam.push_back(frame.clone());
		}
		
		i=i+1;

		// Gestion du temps alloue pour prendre la panoramique
		if (i == 200) 
		{ 
		    taking_panorama = false;
		    cout<<"Arrêté avec succès"<<endl;
		}
	    }

	    // Fermer la camera après la capture
	    cap.release();


	    //stitching de la panoramique
	    Mat pano=stitching(images_cam);

	    //correction du warping
	    Mat pano_rectangle = warp_correction(pano);
	    
	    //obtenir coordonees GPS
	    float latitude = coordonees_gps.latitude; 
	    float longitude = coordonees_gps.longitude;
	    string coord_GPS="latitude: " + to_string(latitude) + ", longitude: " + to_string(longitude);
	    
	    //ajout du text
	    Size dimensions = pano_rectangle.size();
	    int hauteur = dimensions.height;
	    string nom_photo = request->nom; 
	    putText(pano_rectangle, coord_GPS, Point (10,hauteur-20), FONT_HERSHEY_COMPLEX_SMALL,1.0, Scalar(34,139,34), 2); // pour un font plus gros et lisible FONT_HERSHEY_SIMPLEX
	    putText(pano_rectangle, nom_photo, Point (10,hauteur-50), FONT_HERSHEY_COMPLEX_SMALL,1.0, Scalar(34,139,34), 2);
		
		
	    //creation du dossier du dossier de panoramas 
	    struct stat fileInfo;
	    string nom_fichier_panorama;
	    bool dossier_exist = stat("ros2_ws/src/rover/rover_video/src/panoramas", &fileInfo) == 0;

	    if (!dossier_exist)
	    {
	    cout << "dossier pas encore cree" << endl;
	    
	    if (mkdir("ros2_ws/src/rover/rover_video/src/panoramas", 0775)==0){ 
	    	cout << "Succesfully created the folder."<< endl;
	    	nom_fichier_panorama =  "ros2_ws/src/rover/rover_video/src/panoramas/" + result_name;
	    	
	     }else if(mkdir("src/rover/rover_video/src/panoramas", 0775)==0){ 
	     	nom_fichier_panorama =  "src/rover/rover_video/src/panoramas/" + result_name;
	     	
	     }else{
	     	cout << "Failed to create folder" << endl;
	     	nom_fichier_panorama = result_name;
	     	imwrite(result_name,pano_rectangle);//enregistre quand meme mais potentielement hors folder
	     }
	    }else{
	     nom_fichier_panorama =  "ros2_ws/src/rover/rover_video/src/panoramas/" + result_name;
	    }
	    
						
	    //enregistrement de la panoramique
    	    imwrite(nom_fichier_panorama, pano_rectangle);
    
    	    cout << "Panorama done" << endl;
    
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
  

