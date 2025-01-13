#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>


namespace ArucoDetectionHelpers

    
    {
    constexpr int MAX_SAME_FRAME_IDS = 20;
    constexpr cv::aruco::PREDEFINED_DICTIONARY_NAME DICT = cv::aruco::DICT_4X4_250;

    enum CameraAccessMode {URL=0, ID=1};
    
        class ImageCapture
        {
            private:
                std::string cameraURL;
                int cameraID;
                int cameraAccessMode;
                cv::VideoCapture cap;
            
            public:
                
                ImageCapture(std::string _cameraURL, int _cameraAccessMode);
                ImageCapture(int _cameraID, int _cameraAccessMode);

                ~ImageCapture();

                void setCameraURL(std::string);
                std::string getCameraURL();

                bool accessStream();
                bool manageStream();
                cv::Mat getFrame(); 
                void getErrorFrame(cv::Mat&);
                void showFrame();
        };

        
        
        
        class FrameProcessing
        {
            private:
                
                ImageCapture stream;

                int detectedIds[MAX_SAME_FRAME_IDS];
                int same_frame_ids;

                std::vector<std::vector<cv::Point2f>> corners;
                cv::Mat ids;
                cv::Ptr<cv::aruco::Dictionary> dictionary;
                cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

            public:

                FrameProcessing(std::string _cameraURL, int _cameraAccessMode);
                FrameProcessing(int _cameraID, int _cameraAccessMode);
                ~FrameProcessing();
                bool processFrame();
                void showProcessedFrame();

        };
    }

#endif