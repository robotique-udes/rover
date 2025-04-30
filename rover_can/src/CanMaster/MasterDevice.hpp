#ifndef MASTER_DEVICE_HPP
#define MASTER_DEVICE_HPP

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Interface for all CanDevice managed by can master
 *
 */
class MasterDevice
{
  public:
    void attachNode(std::shared_ptr<rclcpp::Node> node_);

  protected:
    /**
     * @brief Method where publisher subs and timers should be created/inited (if persistant)
     *
     */
    virtual void rosElementInit(void) = 0;

    /**
     * @brief Method where ALL ROS2 elements (ptr) should be deleted/reseted.
     *
     */
    virtual void rosElementClean(void) = 0;

    std::shared_ptr<rclcpp::Node> _rosNode;

  private:
    void detachNode(void);
};

#endif  // MASTER_DEVICE_HPP
