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

CameraNode::CameraNode():
    Node("media_server")
{
    _srv_control = this->create_service<rover_msgs::srv::CameraControl>(
        "/rover/video/media_server",
        [this](const std::shared_ptr<rover_msgs::srv::CameraControl::Request> request,
               std::shared_ptr<rover_msgs::srv::CameraControl::Response> response)
        {
            if (!request || !response)
            {
                RCLCPP_ERROR(this->get_logger(), "NULL request or response received.");
                response->success = false;
                response->status = "Service call with null request or response. Possible internal ROS2 error.";
                return;
            }
            this->controlIPCam(*request, *response);
        });

    _msg_position
        = this->create_subscription<rover_msgs::msg::GpsPosition>("/rover/gps/position",
                                                                  1,
                                                                  [this](const rover_msgs::msg::GpsPosition& gps_message)
                                                                  {
                                                                      this->callbackPosition(gps_message);
                                                                  });
}

void CameraNode::controlIPCam(const rover_msgs::srv::CameraControl::Request& request,
                              rover_msgs::srv::CameraControl::Response& response)
{
    RCLCPP_DEBUG(LOGGER, "Entering the controlIPCam function");

    std::string folderPath;
    std::string captureName;
    std::string cameraURL = request.camera_url;

    switch (request.command)
    {
        case rover_msgs::srv::CameraControl::Request::TAKE_PICTURE:
            captureName = getFileName(request.capture_name, cameraURL, SCREENSHOT);
            folderPath = getFolderPath(SCREENSHOT);
            if (!createFolder(folderPath))
            {
                RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
                response.success = false;
                response.status = "Failed to create screenshots folder or it already exists.";
                break;
            }

            if (getScreenshot(folderPath, captureName, cameraURL))
            {
                response.success = true;
                response.status = "Screenshot saved as " + folderPath + "/" + captureName;
            }
            else
            {
                response.success = false;
                response.status = "Failed to take a screenshot.";
            }
            break;

        case rover_msgs::srv::CameraControl::Request::START_RECORDING:
            captureName = getFileName(request.capture_name, cameraURL, VIDEO);
            folderPath = getFolderPath(VIDEO);
            if (!createFolder(folderPath))
            {
                RCLCPP_ERROR(LOGGER, "Failed to create screenshots folder or it already exists.");
                response.success = false;
                response.status = "Failed to create screenshots folder or it already exists.";
                break;
            }
            if (newRecording(folderPath, captureName, cameraURL))
            {
                response.success = true;
                response.status = "Recording started";
            }
            else
            {
                response.success = false;
                response.status = "Failed to take a video.";  // add reason i.e. recording already started at TIME-GPS-NAME
            }

            break;

        case rover_msgs::srv::CameraControl::Request::STOP_RECORDING:
            if (stopRecording(cameraURL))
            {
                response.success = true;
                response.status = "Recording ended";
            }
            else
            {
                response.success = false;
                response.status = "No such recordings";
            }
            break;

        default:
            RCLCPP_INFO(LOGGER, "Invalid command.");
            response.success = false;
            response.status = "Invalid command.";
            break;
    }
}
