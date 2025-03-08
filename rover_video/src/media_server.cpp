#include "screenshot_server.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("media_server"), "Node initialized.");

    try
    {
        rclcpp::spin(std::make_shared<CameraNode>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("Dead Node"), "Killing node on exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

CameraNode::CameraNode(): Node("media_server")
{
    _srv_recording = this->create_service<rover_msgs::srv::CameraControl>(
        "/rover/video/media_server",
        [this](const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request,
               std::shared_ptr<rover_msgs::srv::CameraControl::Response> response) { this->controlIPCam(request, response); });
}

void CameraNode::controlIPCam(const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request,
                              std::shared_ptr<rover_msgs::srv::CameraControl::Response> response)
{
    RCLCPP_INFO(LOGGER, "Entering the controlIPCam function");

    std::string folderPath;
    std::string captureName;
    std::string cameraURL = request->camera_url;
    // Définir screenshot ou video
    switch (request->command)
    {
        case rover_msgs::srv::CameraControl::Request::TAKE_PICTURE:
            captureName = getFileName(request->capture_name, cameraURL, SCREENSHOT);
            folderPath = getFolderPath(SCREENSHOT);
            if (!createFolder(folderPath))
            {
                RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
                response->success = false;
                response->status = "Failed to create screenshots folder or it already exists.";
                break;
            }
            if (getScreenshot(folderPath, captureName, cameraURL))
            {
                response->success = true;
                response->status = "Screenshot saved as " + folderPath + "/" + captureName;
            }
            else
            {
                response->success = false;
                response->status = "Failed to take a screenshot.";
            }
            break;

        case rover_msgs::srv::CameraControl::Request::START_RECORDING:
            captureName = getFileName(request->capture_name, cameraURL, VIDEO);
            folderPath = getFolderPath(VIDEO);
            if (!createFolder(folderPath))
            {
                RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
                response->success = false;
                response->status = "Failed to create screenshots folder or it already exists.";
                break;
            }
            if (newRecording(folderPath, captureName, cameraURL))
            {
                response->success = true;
                response->status = "Recording started";
            }
            else
            {
                response->success = false;
                response->status = "Failed to take a video."; //add reason i.e. recording already started at TIME-GPS-NAME
            }

                break;
        case rover_msgs::srv::CameraControl::Request::STOP_RECORDING:
            if(stopRecording(cameraURL))
            {
                response->success = true;
                response->status = "Recording ended";
            }
            else
            {
                response->success = false;
                response->status = "No such recordings";
            }
                break;

        default:
            RCLCPP_INFO(LOGGER, "Invalid command.");
            response->success = false;
            response->status = "Invalid command.";
            break;
    }
}

/*void CameraNode::recordingIPCam(const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request,
                                std::shared_ptr<rover_msgs::srv::CameraControl::Response> response)
{
    // Select the correct URL using the internal function
    std::string cameraURL = selectCameraURL(request->cameraID);

    // Use the provided file name or a default name
    std::string filename = request->file_name.empty() ? get_current_time() + "_recording.avi" : request->file_name;
    std::string filePath = recordingFolderPath + "/" + filename;

    cv::VideoCapture cap(cameraURL);

    if (!createFolder(recordingFolderPath))
    {
        RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
        response->success = false;
    }

    if (!cap.isOpened())
    {
        RCLCPP_ERROR(LOGGER, "Failed to open camera stream.");
        response->success = false;
    }*/

// Get frame width and height
/* ChatGPT gave me this, gotta look into it more */
// int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
// int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
// int fps = static_cast<int>(cap.get(cv::CAP_PROP_FPS));

// Define the codec and create a VideoWriter object
/* Also from ChatGPT --> more information on OpenCV
--> https://docs.opencv.org/4.x/dd/d9e/classcv_1_1VideoWriter.html */ /*
    cv::VideoWriter video_writer(filePath,
                                 cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                 (fps > 0 ? fps : 30),
                                 cv::Size(frame_width, frame_height));

    if (!video_writer.isOpened())
    {
        RCLCPP_ERROR(LOGGER, "Error: Could not open the output video file for writing!");
        response->success = false;
    }

    RCLCPP_INFO(LOGGER, "Recording... Press 'q' to stop.");

    cv::Mat frame;
    for (EVER)
    {
        cap >> frame;
        if (frame.empty())
        {
            RCLCPP_ERROR(LOGGER, "Error: Blank frame grabbed!");
            break;
        }

        // Write frame to the output video file
        video_writer.write(frame);

        // Show the frame
        cv::imshow("IP Camera Stream", frame);

        // Press 'q' to exit
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    // Release resources
    cap.release();
    video_writer.release();
    cv::destroyAllWindows();

    RCLCPP_INFO(LOGGER, "Recording stopped.");
}*/