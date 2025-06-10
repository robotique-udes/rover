#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/gps.hpp>
#include <rover_msgs/msg/propulsion_motor.hpp>
#include "rover_lib2/helpers/constants.hpp"
#include <rover_lib2/helpers/constants.hpp>

class RoverSim : public rclcpp::Node
{
    static constexpr uint64_t PUBLISHER_PERIOD_MS = 200UL;
    static constexpr float TRACK_WIDTH = 1.0F;
    static constexpr float EARTH_RADIUS_METERS = 6'378'137.0F;

  public:
    RoverSim():
        Node("rover_sim")
    {
        gpsPub_ = this->create_publisher<rover_msgs::msg::Gps>("/rover/gps/position", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_PERIOD_MS), std::bind(&RoverSim::pubGps, this));
        propulsionMotorSub_
            = this->create_subscription<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/wheels_cmd_auto",
                                                                          10,
                                                                          [this](const rover_msgs::msg::PropulsionMotor& msg)
                                                                          {
                                                                              this->CB_prop(msg);
                                                                          });
        currentGps_ = this->create_subscription<rover_msgs::msg::Gps>("/rover/gps/position",
                                                                      10,
                                                                      [this](const rover_msgs::msg::Gps& msg)
                                                                      {
                                                                          this->CB_gps(msg);
                                                                      });
    }

  private:
    void pubGps(void)
    {
        // Create a new GPS message
        auto msg = rover_msgs::msg::Gps();

        float dt = static_cast<float>(PUBLISHER_PERIOD_MS) / 1000.0F;

        float speedRight = (frontRightTargetSpeed_ + rearRightTargetSpeed_) / 2.0F;
        float speedLeft = (frontLeftTargetSpeed_ + rearLeftTargetSpeed_) / 2.0F;

        float linearSpeed = (speedRight + speedLeft) / 2.0F;
        float angularSpeed = (speedRight - speedLeft) / TRACK_WIDTH;

        float distance = linearSpeed * dt;
        float deltaHeading = angularSpeed * dt;

        currentHeading_ += deltaHeading;

        float currentLatRad = currentLatitude_ * M_PI / 180.0F;

        float newLatRad = currentLatRad + (distance / EARTH_RADIUS_METERS) * cos(currentHeading_);
        float newLonRad
            = currentLongitude_ * M_PI / 180.0F + (distance / EARTH_RADIUS_METERS) * sin(currentHeading_) / cos(currentLatRad);

        msg.latitude = newLatRad * 180.0F / M_PI;
        msg.longitude = newLonRad * 180.0F / M_PI;
        msg.heading = currentHeading_ * 180.0F / M_PI;

        gpsPub_->publish(msg);

        currentLatitude_ = msg.latitude;
        currentLongitude_ = msg.longitude;
        currentHeading_ = msg.heading * M_PI / 180.0F;
    }
    void CB_prop(const rover_msgs::msg::PropulsionMotor& msg)
    {
        frontLeftTargetSpeed_ = msg.target_speed[msg.MOTOR_FRONT_LEFT];
        frontRightTargetSpeed_ = msg.target_speed[msg.MOTOR_FRONT_RIGHT];
        rearLeftTargetSpeed_ = msg.target_speed[msg.MOTOR_REAR_LEFT];
        rearRightTargetSpeed_ = msg.target_speed[msg.MOTOR_REAR_RIGHT];

        RCLCPP_INFO(this->get_logger(),
                    "Received propulsion command: FL=%.2f, FR=%.2f, RL=%.2f, RR=%.2f",
                    frontLeftTargetSpeed_,
                    frontRightTargetSpeed_,
                    rearLeftTargetSpeed_,
                    rearRightTargetSpeed_);
    }

    void CB_gps(const rover_msgs::msg::Gps& msg)
    {
        currentLatitude_ = msg.latitude;
        currentLongitude_ = msg.longitude;
        currentHeading_ = msg.heading * M_PI / 180.0F;
    }

    rclcpp::Publisher<rover_msgs::msg::Gps>::SharedPtr gpsPub_;
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr currentGps_;
    rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr propulsionMotorSub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float frontLeftTargetSpeed_;
    float frontRightTargetSpeed_;
    float rearLeftTargetSpeed_;
    float rearRightTargetSpeed_;

    double currentLatitude_ = 45.4043F;
    double currentLongitude_ = -71.8937F;
    float currentHeading_ = 0.0F;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverSim>());
    rclcpp::shutdown();
    return 0;
}
