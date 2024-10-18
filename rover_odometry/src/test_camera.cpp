#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/macros.h"
#include "rovus_lib/timer.hpp"


class VideoTest : public rclcpp::Node
{
    public:
        VideoTest();
        
    
    private:
        void sparseDetection();
        void denseDetection();
};

VideoTest::VideoTest() : Node("video_test")
{
    sparseDetection();
    //denseDetection();
}

void VideoTest::sparseDetection()
{
    std::string filename = GET_PACKAGE_SOURCE_DIR("rover_odometry") + "/src/output.mp4";
    cv::VideoCapture capture(filename);
    if (!capture.isOpened())
    {
        //error in opening the video input
        std::cerr << "Unable to open file!" << std::endl;
        return;
    }

    // Create some random colors
    // std::vector<cv::Scalar> colors;
    // cv::RNG rng;
    // for(int i = 0; i < 100; i++)
    // {
    //     int r = rng.uniform(0, 256);
    //     int g = rng.uniform(0, 256);
    //     int b = rng.uniform(0, 256);
    //     colors.push_back(cv::Scalar(r,g,b));
    // }
    cv::Mat old_frame, old_gray;
    std::vector<cv::Point2f> p0, p1;

    // Take first frame and find corners in it
    capture >> old_frame;
    cv::cvtColor(old_frame, old_gray, cv::COLOR_BGR2GRAY);
    cv::goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);

    // Create a mask image for drawing purposes
    // cv::Mat mask = cv::Mat::zeros(old_frame.size(), old_frame.type());

    int new_points_interval = 1000;

    RoverLib::Timer<uint64_t, RoverLib::millis> _point_timer(new_points_interval);

    while(true)
    {
        cv::Mat frame, frame_gray;
        capture >> frame;
        if (frame.empty())
            break;
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        // calculate optical flow
        std::vector<uchar> status;
        std::vector<float> err;
        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, cv::Size(15,15), 2, criteria);

        std::vector<cv::Point2f> good_new, point_vector;

        for(uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if(status[i] == 1) {
                good_new.push_back(p1[i]);
                point_vector.push_back(p1[i] - p0[i]);
                // draw the tracks
                // cv::line(mask,p1[i], p0[i], colors[i], 2);
                // cv::circle(frame, p1[i], 5, colors[i], -1);
            }
        }

        cv::Scalar mean = cv::mean(point_vector);
        
        RCLCPP_INFO(this->get_logger(), "Mean: %f %f", mean[0], mean[1]);

        // cv::Mat img;
        // cv::add(frame, mask, img);
        // cv::imshow("Frame", img);
        // int keyboard = cv::waitKey(30);
        // if (keyboard == 'q' || keyboard == 27)
        //     break;
        

        // Now update the previous frame and previous points
        old_gray = frame_gray.clone();

        if (_point_timer.isDone())
        {
            cv::goodFeaturesToTrack(old_gray, good_new, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);
            // mask = cv::Mat::zeros(old_frame.size(), old_frame.type());
        }
        else if (good_new.size() == 0)
        {
            cv::goodFeaturesToTrack(old_gray, good_new, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);
            // mask = cv::Mat::zeros(old_frame.size(), old_frame.type());
        }
        p0 = good_new;
        
        
    }
}

void VideoTest::denseDetection()
{
    std::string filename = GET_PACKAGE_SOURCE_DIR("rover_odometry") + "/src/output.mp4";
    cv::VideoCapture capture(filename);
    if (!capture.isOpened())
    {
        //error in opening the video input
        std::cerr << "Unable to open file!" << std::endl;
        return;
    }
    cv::Mat frame1, prvs;
    capture >> frame1;
    cv::cvtColor(frame1, prvs, cv::COLOR_BGR2GRAY);


    while(true){
        cv::Mat frame2, next;
        capture >> frame2;
        if (frame2.empty())
            break;
        cv::cvtColor(frame2, next, cv::COLOR_BGR2GRAY);
        cv::Mat flow(prvs.size(), CV_32FC2);
        cv::calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

        cv::Scalar flowMean;
        flowMean = cv::mean(flow);
        //std::cout << flowMean << std::endl;
        RCLCPP_INFO(this->get_logger(), "Flow mean: %f %f", flowMean[0], flowMean[1]);

        rclcpp::spin_some(this->get_node_base_interface());

        // visualization
        cv::Mat flow_parts[2];
        cv::split(flow, flow_parts);
        cv::Mat magnitude, angle, magn_norm;
        cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
        angle *= ((1.f / 360.0f) * (180.f / 255.f));

        //build hsv image
        cv::Mat _hsv[3], hsv, hsv8, bgr;
        _hsv[0] = angle;
        _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magn_norm;
        cv::merge(_hsv, 3, hsv);
        hsv.convertTo(hsv8, CV_8U, 255.0);
        cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);
        cv::imshow("frame2", bgr);
        
        int keyboard = cv::waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;

        
        prvs = next;
    }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoTest>());
  rclcpp::shutdown();
  return 0;
}