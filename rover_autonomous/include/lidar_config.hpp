#ifndef LIDAR_CONFIG_HPP
#define LIDAR_CONFIG_HPP

namespace LIDAR_CONFIG
{
    namespace COSTMAP
    {
        static constexpr int MAP_WIDTH = 200;
        static constexpr int MAP_HEIGHT = 200;
        static constexpr double MAP_RESOLUTION = 0.05F;
        static constexpr double MAX_OBSTACLE_HEIGHT = 2.0F;
        static constexpr double MIN_OBSTACLE_HEIGHT = 0.1F;
        static constexpr double MAX_RANGE = 10.0F;
        static constexpr double OCCUPIED_THRESHOLD = 0.65F;
        static constexpr double FREE_THRESHOLD = 0.25F;
        static constexpr const char* BASE_FRAME = "unilidar_lidar";
    }  // namespace COSTMAP

    namespace TF
    {
        static constexpr bool PUBLISH_STATIC_TRANSFORM = false;
        static constexpr double LIDAR_ROLL = 0.0F;
        static constexpr double LIDAR_PITCH = 0.0F;
        static constexpr double LIDAR_YAW = 0.0F;
        static constexpr double LIDAR_X = 0.0F;
        static constexpr double LIDAR_Y = 0.0F;
        static constexpr double LIDAR_Z = 0.5F;
    }  // namespace TF
}  // namespace LIDAR_CONFIG

#endif