#ifndef MASTER_DEVICE_HPP
#define MASTER_DEVICE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rover_can2/constant.hpp>

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

    virtual std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) = 0;

    std::shared_ptr<rclcpp::Node> getAttachedNode(void)
    {
        return _rosNode;
    }

  private:
    std::shared_ptr<rclcpp::Node> _rosNode;

    void detachNode(void);
};

#endif  // MASTER_DEVICE_HPP
