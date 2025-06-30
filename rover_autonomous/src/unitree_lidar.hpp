#ifndef UNITREE_LIDAR_HPP
#define UNITREE_LIDAR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class UnitreeLidar : public rclcpp::Node
{
    static constexpr const char* TOPIC_LIDAR_POINT_CLOUD = "/unilidar/cloud";
    static constexpr const char* TOPIC_COSTMAP = "/rover/autonomous/costmap";

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_pointCloud;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _pub_costmap;

    nav_msgs::msg::OccupancyGrid _costmap;

  public:
    UnitreeLidar();
    ~UnitreeLidar() = default;

    void CB_pointCloud(const sensor_msgs::msg::PointCloud2& pointCloudMsg_);
    void filterPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void updateCostmap(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    void initCostmap(void);
    void rayTrace(int sensorX_, int sensorY_, int obstacleX_, int obstacleY_);
};

#endif