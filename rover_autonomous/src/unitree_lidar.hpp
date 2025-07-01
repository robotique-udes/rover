#ifndef UNITREE_LIDAR_HPP
#define UNITREE_LIDAR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class UnitreeLidar : public rclcpp::Node
{
    static constexpr const char* TOPIC_LIDAR_POINT_CLOUD = "/unilidar/cloud";
    static constexpr const char* TOPIC_COSTMAP = "/rover/autonomous/costmap";

    static constexpr int OCCUPIED_CELL = 100;

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_pointCloud;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _pub_costmap;

    nav_msgs::msg::OccupancyGrid _costmap;

    // Transform-related members
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_broadcaster;

    std::string _base_frame; // TODO fix this

  public:
    UnitreeLidar();
    ~UnitreeLidar() = default;

    void CB_pointCloud(const sensor_msgs::msg::PointCloud2& pointCloudMsg_);
    void filterPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void updateCostmap(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    void initCostmap(void);
    void rayTrace(int sensorX_, int sensorY_, int obstacleX_, int obstacleY_);

    void setupStaticTransform(const std::string& lidar_frame,
                              double x,
                              double y,
                              double z,
                              double roll_deg,
                              double pitch_deg,
                              double yaw_deg);
    bool transformPointCloud(const sensor_msgs::msg::PointCloud2& input_cloud, sensor_msgs::msg::PointCloud2& output_cloud);
};

#endif