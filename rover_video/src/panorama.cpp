#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/photo_panoramique.hpp"
#include "rover_msgs/srv/photo_panoramique.hpp"
#include <iostream>
#include <vector>


using namespace std;
using namespace cv;

class PhotoPanoramique : public rclcpp::Node
{
  public:
    PhotoPanoramique();
    
    Mat warp_correction(Mat pano) {

	    Size dimensions = pano.size();
	    
	    int width = dimensions.width;
	    int hauteur = dimensions.height;
	    
	    Rect coupe(50, 50, width-100, hauteur-100);
	    Mat pano_rectangle = pano(coupe);
	    
	    return pano_rectangle;
	}

     Mat stitching(vector<Mat> images_cam)  {
	    Mat pano;
	    cout<<"Maintenant en essai de stitching"<<endl;
	    Ptr<Stitcher> stitcher = Stitcher::create(Stitcher::PANORAMA);
	    stitcher->stitch(images_cam, pano);
	 
	     return pano;
	}

  private:
    rclcpp::Publisher<rover_msgs::msg::PhotoPanoramique>::SharedPtr _pubpanorama;
    rclcpp::TimerBase::SharedPtr _timerPub;
    rclcpp::Service<rover_msgs::srv::PhotoPanoramique>::SharedPtr _srvpanorama;
    
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
   VideoCapture cap;
   string path_camera= "/dev/video0";        
   int apiID = cv::CAP_ANY; 
   cap.open(path_camera, apiID);
   
    cout << "camera open" << endl;


   //paramètres pour le traitement des images
    Mat frame;
    vector<Mat> images_cam;
    bool taking_panorama = true;

    cout << "Début de la capture vidéo pour la panoramique\nAppuyez sur esc pour arrêter" << endl;
   
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

        // Gestion du clavier
        if (i == 200) 
        { 
            taking_panorama = false;
            cout<<"Arrêté avec succès"<<endl;
        }
    }

    // Fermer la fenêtre après la capture
    cap.release();


    //stitching de la panoramique
    Mat pano=stitching(images_cam);

    //correction du warping
    Mat pano_rectangle = warp_correction(pano);
    
    //ajout du text
    Size dimensions = pano_rectangle.size();
    int hauteur = dimensions.height;
    char nom_photo = request->nom;
    putText(pano_rectangle, "coordonees GPS", Point (10,hauteur-20), FONT_HERSHEY_COMPLEX_SMALL,1.0, Scalar(255,0,0), 2); // pour un font plus gros et lisible FONT_HERSHEY_SIMPLEX
    putText(pano_rectangle, "nom_photo", Point (10,hauteur-50), FONT_HERSHEY_COMPLEX_SMALL,1.0, Scalar(255,0,0), 2);
        
    //display de la panoramique
    imwrite(result_name, pano_rectangle);
    
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
  

