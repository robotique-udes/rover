#include "aruco_detection.h"

using namespace ArucoDetectionHelpers;

//useful functions for outsiders: .getFrame

ImageCapture::ImageCapture(std::string _cameraURL, int _cameraAccessMode)
{
    cameraURL = _cameraURL;
    cameraAccessMode = _cameraAccessMode;
}


ImageCapture::ImageCapture(int _cameraID, int _cameraAccessMode):cameraID(_cameraID), cameraAccessMode(_cameraAccessMode) {}



ImageCapture::~ImageCapture()
{
    cap.release();
    cv::destroyAllWindows();
}


void ImageCapture::setCameraURL(std::string URL)
{
    cameraURL = URL;
}


std::string ImageCapture::getCameraURL()
{
    return cameraURL;
}


bool ImageCapture::accessStream()
{

    if(cameraAccessMode == CameraAccessMode::URL)
    {
        cap.open(cameraURL);
    }

    else
    {
        cap.open(cameraID);
    }

    return cap.isOpened();
}


bool ImageCapture::manageStream()
{
    if(cap.isOpened())
    {
        return true;
    }
    else 
    {
        return accessStream();
    }
}


cv::Mat ImageCapture::getFrame()
{
    cv::Mat frame;
    if(!cap.isOpened())
    {
        if(!manageStream())
        {
            getErrorFrame(frame);
            return frame;
        } 
    }

    cap >> frame;

    if (frame.empty()) getErrorFrame(frame);
    return frame;


}


void ImageCapture::getErrorFrame(cv::Mat& frame)
{
    frame = cv::Mat::zeros(480, 640, CV_8UC3); 
    std::string error_message = "Error: Stream not found!";
    cv::putText(frame, error_message, cv::Point(100, 240), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
}


void ImageCapture::showFrame()
{
    cv::imshow("frame",getFrame());
    cv::waitKey(0);
}



                

FrameProcessing::FrameProcessing(std::string _cameraURL, int _cameraAccessMode) : stream(_cameraURL, _cameraAccessMode)
{
    dictionary = cv::aruco::getPredefinedDictionary(DICT);
    detectorParams = cv::aruco::DetectorParameters::create();
}

FrameProcessing::FrameProcessing(int _cameraID, int _cameraAccessMode) : stream(_cameraID, _cameraAccessMode)
{
    dictionary = cv::aruco::getPredefinedDictionary(DICT);
    detectorParams = cv::aruco::DetectorParameters::create();
}


FrameProcessing::~FrameProcessing() {}



bool FrameProcessing::processFrame()
{
    cv::aruco::detectMarkers(stream.getFrame(), dictionary, corners, ids);
    if(ids.empty()) return false;
    return true;
}

void FrameProcessing::showProcessedFrame()
{
    processFrame();
    cv::Mat processedFrame = stream.getFrame();
    cv::aruco::drawDetectedMarkers(processedFrame, corners, ids);
    cv::imshow("Aruco Detection", processedFrame);
    cv::waitKey(0);

}



int main()
{
    FrameProcessing frame(0, ID);
    FrameProcessing frame2(1, ID);
    
    for(int i=0; i<50; i++){
        if(i%10>5)
        {
            frame.showProcessedFrame();
        }

        else
        {
            frame2.showProcessedFrame();
        }
    }
    return 0;
}
