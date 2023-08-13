#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "rover_control_msgs/camera_control.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "sensor_msgs/JointState.h"

#define IN
#define OUT
#define INOUT

// Needs to be params
// #define CAM_TOPIC "/cam_sonix/image"
#define NB_PICS 4

bool takePicture(std::string topic_name, cv_bridge::CvImagePtr img);
bool CBPanorama(rover_control_msgs::camera_controlRequest &req, rover_control_msgs::camera_controlResponse &res);
std::vector<int> getMarkerId(cv::Mat img);
void normalize_rad(INOUT double *angle);

ros::NodeHandle *p_nh;

std::list<cv::Mat> l_img;
ros::ServiceServer srv_cmd_server;
sensor_msgs::JointState joint_state;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_control_server");
    ros::NodeHandle nh;
    p_nh = &nh;

    srv_cmd_server = nh.advertiseService<rover_control_msgs::camera_controlRequest,
                                         rover_control_msgs::camera_controlResponse>("/camera_control_server",
                                                                                     CBPanorama);

    while (!ros::isShuttingDown())
    {
        ros::spinOnce();
        ros::Duration(1.0f).sleep();
    }

    return 0;
}

bool takePicture(IN std::string topic_name, OUT cv::Mat *img)
{
    const sensor_msgs::ImageConstPtr sp_img = ros::topic::waitForMessage<sensor_msgs::Image>(topic_name, *p_nh, ros::Duration(5.0f));
    if (sp_img == NULL)
    {
        ROS_ERROR("Error taking picture");
        return false;
    }

    try
    {
        cv_bridge::CvImagePtr ros_img;
        *img = cv_bridge::toCvCopy(sp_img, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

    return true;
}

bool CBPanorama(rover_control_msgs::camera_controlRequest &req, rover_control_msgs::camera_controlResponse &res)
{
    res.result = false;
    switch (req.cmd)
    {
    case rover_control_msgs::camera_controlRequest::CMD_TAKE_PICTURE:
    {
        cv::Mat img;
        if (!takePicture(IN req.cam_topic, OUT & img))
        {
            res.result = false;
            break;
        }

        l_img.push_back(img);

        std::vector<int> marker_ids = getMarkerId(img);

        int detected_marker_nb = static_cast<int>(marker_ids.size());
        ROS_INFO("Picture #%d taken. %d aruco marker detected", static_cast<int>(l_img.size()) - 1, detected_marker_nb);
        if (static_cast<int>(marker_ids.size()) != 0)
        {
            std::string str_ids = "";

            for (int i = 0; i < detected_marker_nb; i++)
            {
                str_ids.append(std::to_string(marker_ids.back()) + ", ");
                marker_ids.pop_back();
            }

            ROS_INFO("Aruco marker detected: %s", str_ids.c_str());
        }

        res.result = true;
    }
    break;

    case rover_control_msgs::camera_controlRequest::CMD_SAVE_PANO:
    {
        int nb_pic = static_cast<int>(l_img.size());

        if (nb_pic == 0)
        {
            ROS_ERROR("Cannot create panorama with 0 photos");
            res.result = false;
            break;
        }

        ROS_INFO("Creating panorama with %d photos", nb_pic);
        cv::Mat imgs[nb_pic];
        for (int i = 0; i < nb_pic; i++)
        {
            imgs[i] = l_img.front();
            l_img.pop_front();
        }

        cv::Mat img_panorama;
        cv::hconcat(imgs, nb_pic, img_panorama);

        std::string path = ros::package::getPath("rover_control");
        boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
        std::string time_stamp = boost::posix_time::to_iso_extended_string(my_posix_time);
        std::replace(time_stamp.begin(), time_stamp.end(), ':', '-');

        path.append("/img/panorama" + time_stamp + ".png");

        if (!cv::imwrite(path, img_panorama))
        {
            ROS_FATAL("Error when saving Panorama");
            exit(1);
        }
        ROS_INFO("Panorama as been save: %s ", path.c_str());

        res.result = true;
        break;
    }

    case rover_control_msgs::camera_controlRequest::CMD_RESET_PICTURES:
    {
        ROS_INFO("Deleting all photos");
        l_img = std::list<cv::Mat>();
        ROS_INFO("Number of photo left: %d", static_cast<int>(l_img.size()));

        res.result = true;
        break;
    }

    case rover_control_msgs::camera_controlRequest::CMD_DETECT_ARUCO:
    {
        cv::Mat img;
        if (!takePicture(IN req.cam_topic, OUT & img))
        {
            res.result = false;
            break;
        }

        std::vector<int> marker_ids = getMarkerId(img);

        for (; static_cast<int>(marker_ids.size()) != 0;)
        {
            res.detected_aruco_marker.push_back(static_cast<uint8_t>(marker_ids.back()));
            marker_ids.pop_back();
        }

        res.result = true;
        break;
    }

    case rover_control_msgs::camera_controlRequest::CMD_TAKE_AND_SAVE_PANO:
    {
        sensor_msgs::JointState joint_state;
        boost::shared_ptr<const sensor_msgs::JointState> ptr_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", *p_nh);
        if (ptr_joint_state != NULL)
        {
            joint_state = *ptr_joint_state;
        }
        else
        {
            res.result = false;
            break;
        }

        ros::Publisher pub = p_nh->advertise<sensor_msgs::JointState>("/desired_joint_states", 1);

        ROS_INFO("Starting Panorama...");
        ROS_INFO("Waiting 1 seconds for publisher subscription...");
        ros::Duration(1.0).sleep();

        // Get motor index
        uint8_t index = 0;
        for (; static_cast<uint8_t>(joint_state.name.size()); index++)
        {
            if (joint_state.name[index] == "module_science")
                break;
        }

        double target_increments = 6.28318530718 / NB_PICS;
        double target = 0.00;
        cv::Mat a_img_pano[NB_PICS];
        for (uint8_t i = 0; i < NB_PICS; i++, target += target_increments)
        {
            ROS_INFO("Taking pic #%d...", i);
            // Goes to next position
            sensor_msgs::JointState desired_joint_state;
            desired_joint_state.name.push_back("module_science");
            desired_joint_state.velocity.push_back(5.0f);
            pub.publish(desired_joint_state);

            uint16_t max_loop = 0;
            double normalized_position = joint_state.position[index];
            normalize_rad(&normalized_position);
            for (; !(normalized_position <= (target + 0.2f) && normalized_position >= (target - 0.2f)) && max_loop < 65000; max_loop++)
            {
                ROS_WARN("Current normalized position: #%f | Target is: %f", normalized_position, target);
                boost::shared_ptr<const sensor_msgs::JointState> ptr_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", *p_nh);
                if (ptr_joint_state != NULL)
                {
                    joint_state = *ptr_joint_state;
                    normalized_position = abs(joint_state.position[index]);
                    normalize_rad(&normalized_position);
                }
            }
            // Stop joint
            desired_joint_state = sensor_msgs::JointState();
            desired_joint_state.name.push_back("module_science");
            desired_joint_state.velocity.push_back(0.0f);
            pub.publish(desired_joint_state);

            cv::Mat img;
            if (!takePicture(IN req.cam_topic, OUT & img))
            {
                res.result = false;
                break;
            }
            a_img_pano[i] = img;
            ros::Duration(2.0).sleep();
        }

        // Goes back to position --0.0--
        sensor_msgs::JointState desired_joint_state;
        desired_joint_state.name.push_back("module_science");
        desired_joint_state.velocity.push_back(-5.0f);
        pub.publish(desired_joint_state);

        double normalized_position = 0.0;
        uint16_t max_loop = 0;
        for (; !(normalized_position <= (target + 0.1f) && normalized_position >= (target - 0.1f)) && max_loop < 65000; max_loop++)
        {
            ROS_WARN("Current normalized position: #%f | Target is: %f", normalized_position, target);
            boost::shared_ptr<const sensor_msgs::JointState> ptr_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", *p_nh);
            if (ptr_joint_state != NULL)
            {
                joint_state = *ptr_joint_state;
                normalized_position = abs(joint_state.position[index]);
                normalize_rad(&normalized_position);
            }
        }

        desired_joint_state = sensor_msgs::JointState();
        desired_joint_state.name.push_back("module_science");
        desired_joint_state.velocity.push_back(0.0f);
        pub.publish(desired_joint_state);

        // Concatenate pano
        std::string path = ros::package::getPath("rover_control");
        boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
        std::string time_stamp = boost::posix_time::to_iso_extended_string(my_posix_time);
        std::replace(time_stamp.begin(), time_stamp.end(), ':', '-');

        path.append("/img/panorama" + time_stamp + ".png");

        cv::Mat img_panorama;
        cv::hconcat(a_img_pano, NB_PICS, img_panorama);
        if (!cv::imwrite(path + "_panorama", img_panorama))
        {
            ROS_ERROR("Can't save picture, path doesn't exist: \"%s\"", path.c_str());
        }
        ROS_INFO("Panorama as been save: %s ", path.c_str());

        for (uint8_t i = 0; i < NB_PICS; i++)
        {
            if (!cv::imwrite(path + "_" + std::to_string(i), img_panorama))
            {
                ROS_ERROR("Can't save picture, path doesn't exist: \"%s\"", path.c_str());
            }
            ROS_INFO("Panorama pic #%d as been save: %s ", i, path.c_str());
        }
        break;
    }

    default:
        ROS_WARN("Invalid cmd in service call, nothing done");
        break;
    }
    return true;
}

std::vector<int> getMarkerId(cv::Mat img)
{
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<std::vector<cv::Point2f>> rejected_candidates;
    cv::aruco::DetectorParameters detectorParam;
    std::vector<int> marker_ids;
    cv::aruco::detectMarkers(img, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100), marker_corners, marker_ids);

    return marker_ids;
}

void normalize_rad(INOUT double *angle)
{
    if (*angle < 0.0f)
    {
        exit(-69);
    }

    for (; *angle > 6.28318530718;)
    {
        *angle -= 6.28318530718f;
    }
}
