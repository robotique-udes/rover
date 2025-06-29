#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudCostmapNode : public rclcpp::Node
{
  public:
    PointCloudCostmapNode():
        Node("pointcloud_costmap_node")
    {
        // Declare parameters
        this->declare_parameter("map_width", 200);
        this->declare_parameter("map_height", 200);
        this->declare_parameter("map_resolution", 0.05);  // meters per pixel
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("max_obstacle_height", 2.0);
        this->declare_parameter("min_obstacle_height", 0.1);
        this->declare_parameter("max_range", 10.0);
        this->declare_parameter("occupied_threshold", 0.65);
        this->declare_parameter("free_threshold", 0.25);

        // Get parameters
        map_width_ = this->get_parameter("map_width").as_int();
        map_height_ = this->get_parameter("map_height").as_int();
        map_resolution_ = this->get_parameter("map_resolution").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        max_obstacle_height_ = this->get_parameter("max_obstacle_height").as_double();
        min_obstacle_height_ = this->get_parameter("min_obstacle_height").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        occupied_threshold_ = this->get_parameter("occupied_threshold").as_double();
        free_threshold_ = this->get_parameter("free_threshold").as_double();

        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize costmap
        initializeCostmap();

        // Create subscriber for point cloud
        pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            TOPIC_LIDAR_POINT_CLOUD,
            10,
            std::bind(&PointCloudCostmapNode::pointCloudCallback, this, std::placeholders::_1));

        // Create publisher for costmap
        costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

        RCLCPP_INFO(this->get_logger(), "PointCloud Costmap Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", TOPIC_LIDAR_POINT_CLOUD);
        RCLCPP_INFO(this->get_logger(), "Publishing costmap to: /costmap");
        RCLCPP_INFO(this->get_logger(), "Base frame: %s", base_frame_.c_str());
    }

  private:
    static constexpr const char* TOPIC_LIDAR_POINT_CLOUD = "/unilidar/cloud";

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    int map_width_, map_height_;
    double map_resolution_;
    std::string base_frame_;
    double max_obstacle_height_, min_obstacle_height_;
    double max_range_;
    double occupied_threshold_, free_threshold_;

    // Costmap data
    nav_msgs::msg::OccupancyGrid costmap_;
    std::vector<int> hit_count_;
    std::vector<int> miss_count_;

    void initializeCostmap()
    {
        costmap_.header.frame_id = base_frame_;
        costmap_.info.resolution = map_resolution_;
        costmap_.info.width = map_width_;
        costmap_.info.height = map_height_;

        // Center the map on the robot
        costmap_.info.origin.position.x = -map_width_ * map_resolution_ / 2.0;
        costmap_.info.origin.position.y = -map_height_ * map_resolution_ / 2.0;
        costmap_.info.origin.position.z = 0.0;
        costmap_.info.origin.orientation.w = 1.0;

        // Initialize data arrays
        int map_size = map_width_ * map_height_;
        costmap_.data.resize(map_size, -1);  // Unknown
        hit_count_.resize(map_size, 0);
        miss_count_.resize(map_size, 0);
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try
        {
            // Transform point cloud to base_link frame
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            if (!transformPointCloud(msg, transformed_cloud))
            {
                return;
            }

            // Convert to PCL format
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(transformed_cloud, *cloud);

            // Filter point cloud
            filterPointCloud(cloud);

            // Update costmap
            updateCostmap(cloud);

            // Publish costmap
            costmap_.header.stamp = this->get_clock()->now();
            costmap_publisher_->publish(costmap_);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
        }
    }

    bool transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                             sensor_msgs::msg::PointCloud2& output_cloud)
    {
        try
        {
            // Wait for transform to be available
            if (!tf_buffer_->canTransform(base_frame_,
                                          input_cloud->header.frame_id,
                                          input_cloud->header.stamp,
                                          rclcpp::Duration::from_seconds(0.1)))
            {
                RCLCPP_WARN(this->get_logger(),
                            "Transform from %s to %s not available",
                            input_cloud->header.frame_id.c_str(),
                            base_frame_.c_str());
                return false;
            }

            // Transform the point cloud
            tf2::doTransform(*input_cloud,
                             output_cloud,
                             tf_buffer_->lookupTransform(base_frame_, input_cloud->header.frame_id, input_cloud->header.stamp));
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return false;
        }
    }

    void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // Filter by height (Z coordinate in base_link frame)
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(min_obstacle_height_, max_obstacle_height_);
        pass_z.filter(*cloud);

        // Filter by range (distance from robot)
        pcl::PassThrough<pcl::PointXYZ> pass_range;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& point : cloud->points)
        {
            double range = sqrt(point.x * point.x + point.y * point.y);
            if (range <= max_range_)
            {
                filtered_cloud->points.push_back(point);
            }
        }

        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = true;

        *cloud = *filtered_cloud;

        // Optional: Downsample with voxel grid
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(map_resolution_ / 2.0, map_resolution_ / 2.0, 0.1);
        voxel_filter.filter(*cloud);
    }

    void updateCostmap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // Clear previous data
        std::fill(hit_count_.begin(), hit_count_.end(), 0);
        std::fill(miss_count_.begin(), miss_count_.end(), 0);

        // Process each point
        for (const auto& point : cloud->points)
        {
            // Convert world coordinates to grid coordinates
            int grid_x = static_cast<int>((point.x - costmap_.info.origin.position.x) / map_resolution_);
            int grid_y = static_cast<int>((point.y - costmap_.info.origin.position.y) / map_resolution_);

            // Check bounds
            if (grid_x >= 0 && grid_x < map_width_ && grid_y >= 0 && grid_y < map_height_)
            {
                int index = grid_y * map_width_ + grid_x;
                hit_count_[index]++;

                // Ray tracing from robot origin to obstacle
                rayTrace(0, 0, grid_x, grid_y);
            }
        }

        // Update costmap based on hit/miss counts
        for (int i = 0; i < static_cast<int>(costmap_.data.size()); ++i)
        {
            int total_count = hit_count_[i] + miss_count_[i];
            if (total_count > 0)
            {
                double occupancy_prob = static_cast<double>(hit_count_[i]) / total_count;

                if (occupancy_prob >= occupied_threshold_)
                {
                    costmap_.data[i] = 100;  // Occupied
                }
                else if (occupancy_prob <= free_threshold_)
                {
                    costmap_.data[i] = 0;  // Free
                }
                else
                {
                    costmap_.data[i] = static_cast<int>(occupancy_prob * 100);  // Uncertain
                }
            }
        }
    }

    void rayTrace(int x0, int y0, int x1, int y1)
    {
        // Bresenham's line algorithm for ray tracing
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;

        while (true)
        {
            // Don't mark the endpoint as free (it's an obstacle)
            if (x != x1 || y != y1)
            {
                if (x >= 0 && x < map_width_ && y >= 0 && y < map_height_)
                {
                    int index = y * map_width_ + x;
                    miss_count_[index]++;
                }
            }

            if (x == x1 && y == y1)
                break;

            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y += sy;
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudCostmapNode>());
    rclcpp::shutdown();
    return 0;
}