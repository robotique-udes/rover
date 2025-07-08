#ifndef LIDAR_CONFIG_HPP
#define LIDAR_CONFIG_HPP

namespace LIDAR_CONFIG
{
    namespace COSTMAP
    {
        static constexpr int MAP_WIDTH = 200;
        static constexpr int MAP_HEIGHT = 200;
        static constexpr double MAP_RESOLUTION = 0.025F;
        static constexpr double MAX_OBSTACLE_HEIGHT = 2.0F;
        static constexpr double MIN_OBSTACLE_HEIGHT = 0.5F;
        static constexpr double MAX_RANGE = 5.0F; //meters
        static constexpr double OCCUPIED_THRESHOLD = 0.80F;
        static constexpr double FREE_THRESHOLD = 0.25F;
        static constexpr const char* BASE_FRAME = "base_link";
    }  // namespace COSTMAP

    namespace TF
    {
        static constexpr bool PUBLISH_STATIC_TRANSFORM = true;
        static constexpr const char* LIDAR_FRAME = "unilidar_lidar";
        static constexpr double LIDAR_ROLL = 0.0F;
        static constexpr double LIDAR_PITCH = 90.0F;
        static constexpr double LIDAR_YAW = 0.0F;
        static constexpr double LIDAR_X = 0.0F;
        static constexpr double LIDAR_Y = 0.0F;
        static constexpr double LIDAR_Z = 0.5F;
    }  // namespace TF

    namespace NAVIGATION
    {
        static constexpr float REPULSIVE_GAIN = 1.0F;
        static constexpr float INFLUENCE_DISTANCE = 1.0F;
    }
}  // namespace LIDAR_CONFIG

#endif