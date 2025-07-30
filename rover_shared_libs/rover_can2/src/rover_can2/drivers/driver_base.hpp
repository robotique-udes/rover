#ifndef ROVER_CAN2_DRIVERS_DRIVER_BASE_HPP
#define ROVER_CAN2_DRIVERS_DRIVER_BASE_HPP

#include "rover_lib2/rover_object.hpp"
#include "rover_can2/can_msg.hpp"
#include <optional>

namespace RoverCan2::Drivers
{
    // Allows template shadowing for type validation
    class DriverBaseT
    {
      protected:
        DriverBaseT() = default;
    };

    template<typename Impl_T>
    class DriverBase : public DriverBaseT
    {
      private:
        friend Impl_T;
        DriverBase() = default;

      public:
        void init(void)
        {
            static_cast<Impl_T*>(this)->_init();
        }

        void update(void)
        {
            static_cast<Impl_T*>(this)->_update();
        }

        bool sendMsg(const CanMsg& msg_)
        {
            return static_cast<Impl_T*>(this)->_sendMsg(msg_);
        }

        std::optional<CanMsg> getMsg(void)
        {
            return static_cast<Impl_T*>(this)->_getMsg();
        }

        size_t getAvailableMessagesNb(void) const
        {
            return static_cast<Impl_T*>(this)->_getAvailableMessagesNb();
        }
    };
}  // namespace RoverCan2::Drivers

#endif  // ROVER_CAN2_DRIVERS_DRIVER_BASE_HPP
