#include "unitree_lidar.hpp"
#include "lidar_config.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <rover_lib2/helpers/constants.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

// TODO Remove M_PI for std::numbers::pi

UnitreeLidar::UnitreeLidar():
    Node("unitree_lidar_node")
{
    // Set frame names from config
    _base_frame = LIDAR_CONFIG::COSTMAP::BASE_FRAME;

    // Initialize TF2
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    // Setup static transform if enabled in config
    if (LIDAR_CONFIG::TF::PUBLISH_STATIC_TRANSFORM)
    {
        setupStaticTransform(LIDAR_CONFIG::TF::LIDAR_FRAME,
                             LIDAR_CONFIG::TF::LIDAR_X,
                             LIDAR_CONFIG::TF::LIDAR_Y,
                             LIDAR_CONFIG::TF::LIDAR_Z,
                             LIDAR_CONFIG::TF::LIDAR_ROLL,
                             LIDAR_CONFIG::TF::LIDAR_PITCH,
                             LIDAR_CONFIG::TF::LIDAR_YAW);
    }

    _sub_pointCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(TOPIC_LIDAR_POINT_CLOUD,
                                                                               QOS_DEFAULT,
                                                                               [this](const sensor_msgs::msg::PointCloud2& pcMsg_)
                                                                               {
                                                                                   this->CB_pointCloud(pcMsg_);
                                                                               });

    _pub_costmap = this->create_publisher<nav_msgs::msg::OccupancyGrid>(TOPIC_COSTMAP, QOS_DEFAULT);

    this->initCostmap();
}

void UnitreeLidar::setupStaticTransform(const std::string& lidar_frame,
                                        double x,
                                        double y,
                                        double z,
                                        double roll_deg,
                                        double pitch_deg,
                                        double yaw_deg)
{
    _tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = _base_frame;
    transform.child_frame_id = lidar_frame;

    // Set translation
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;

    // Convert degrees to radians and set rotation
    tf2::Quaternion q;
    q.setRPY(roll_deg * M_PI / 180.0, pitch_deg * M_PI / 180.0, yaw_deg * M_PI / 180.0);
    q.normalize();

    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    _tf_broadcaster->sendTransform(transform);
}

bool UnitreeLidar::transformPointCloud(const sensor_msgs::msg::PointCloud2& input_cloud,
                                       sensor_msgs::msg::PointCloud2& output_cloud)
{
    if (input_cloud.header.frame_id == _base_frame)
    {
        output_cloud = input_cloud;
        return true;
    }

    if (!_tf_buffer->canTransform(_base_frame,
                                  input_cloud.header.frame_id,
                                  input_cloud.header.stamp,
                                  rclcpp::Duration::from_seconds(0.1)))
    {
        return false;
    }

    tf2::doTransform(input_cloud,
                     output_cloud,
                     _tf_buffer->lookupTransform(_base_frame, input_cloud.header.frame_id, input_cloud.header.stamp));
    return true;
}

void UnitreeLidar::updateCostmap(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    _costmap.data.clear();
    _costmap.data.resize(_costmap.info.width * _costmap.info.height, -1);

    int sensorX_ = static_cast<int>(-_costmap.info.origin.position.x / _costmap.info.resolution);
    int sensorY_ = static_cast<int>(-_costmap.info.origin.position.y / _costmap.info.resolution);

    for (const auto& point : cloud.points)
    {
        int end_x = static_cast<int>((point.x - _costmap.info.origin.position.x) / _costmap.info.resolution);
        int end_y = static_cast<int>((point.y - _costmap.info.origin.position.y) / _costmap.info.resolution);

        if (end_x >= 0 && end_x < _costmap.info.width && end_y >= 0 && end_y < _costmap.info.height)
        {
            this->rayTrace(sensorX_, sensorY_, end_x, end_y);

            int obstacle_index = end_y * _costmap.info.width + end_x;
            _costmap.data[obstacle_index] = OCCUPIED_CELL;
        }
    }

    _costmap.header.stamp = this->now();
    _pub_costmap->publish(_costmap);
}

void UnitreeLidar::CB_pointCloud(const sensor_msgs::msg::PointCloud2& pcMsg_)
{
    // Transform point cloud to base frame if needed
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    if (!transformPointCloud(pcMsg_, transformed_cloud))
    {
        // If transform fails, try to use the original cloud
        transformed_cloud = pcMsg_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

    this->filterPointcloud(pcl_cloud);
    this->updateCostmap(*pcl_cloud);
}

void UnitreeLidar::initCostmap(void)
{
    _costmap.header.frame_id = _base_frame;  // Use base frame instead of hardcoded frame
    _costmap.info.resolution = LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION;
    _costmap.info.width = LIDAR_CONFIG::COSTMAP::MAP_WIDTH;
    _costmap.info.height = LIDAR_CONFIG::COSTMAP::MAP_HEIGHT;
    _costmap.info.origin.position.x = -LIDAR_CONFIG::COSTMAP::MAP_WIDTH * LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION / 2.0;
    _costmap.info.origin.position.y = -LIDAR_CONFIG::COSTMAP::MAP_HEIGHT * LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION / 2.0;
    _costmap.info.origin.position.z = 0.0;
    _costmap.info.origin.orientation.w = 1.0;
}

void UnitreeLidar::filterPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(LIDAR_CONFIG::COSTMAP::MIN_OBSTACLE_HEIGHT, LIDAR_CONFIG::COSTMAP::MAX_OBSTACLE_HEIGHT);
    pass_z.filter(*cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& point : cloud->points)
    {
        double range = sqrt(point.x * point.x + point.y * point.y);
        if (range <= LIDAR_CONFIG::COSTMAP::MAX_RANGE)
        {
            filteredCloud->points.push_back(point);
        }
    }

    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1;
    filteredCloud->is_dense = true;

    *cloud = *filteredCloud;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION / 2.0, LIDAR_CONFIG::COSTMAP::MAP_RESOLUTION / 2.0, 0.1);
    voxel_filter.filter(*cloud);
}

void UnitreeLidar::rayTrace(int sensorX_, int sensorY_, int obstacleX_, int obstacleY_)
{
    int delta_x = abs(obstacleX_ - sensorX_);
    int delta_y = abs(obstacleY_ - sensorY_);
    int step_x = (sensorX_ < obstacleX_) ? 1 : -1;
    int step_y = (sensorY_ < obstacleY_) ? 1 : -1;
    int error = delta_x - delta_y;

    int current_x = sensorX_;
    int current_y = sensorY_;

    while (true)
    {
        if (current_x >= 0 && current_x < _costmap.info.width && current_y >= 0 && current_y < _costmap.info.height)
        {
            int cell_index = current_y * _costmap.info.width + current_x;

            if (_costmap.data[cell_index] == -1)
            {
                _costmap.data[cell_index] = 0;
            }
        }

        if (current_x == obstacleX_ && current_y == obstacleY_)
            break;

        int error_doubled = 2 * error;
        if (error_doubled > -delta_y)
        {
            error -= delta_y;
            current_x += step_x;
        }
        if (error_doubled < delta_x)
        {
            error += delta_x;
            current_y += step_y;
        }
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeLidar>());
    rclcpp::shutdown();
}