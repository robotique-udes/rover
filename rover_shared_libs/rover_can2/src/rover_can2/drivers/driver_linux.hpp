#ifndef DRIVER_LINUX_HPP
#define DRIVER_LINUX_HPP

#include "rover_can2/drivers/driver_base.hpp"

namespace RoverCan2::Drivers
{
    class DriverLinux : public DriverBase<DriverLinux>
    {
      public:
        DriverLinux() = default;

      private:
    };
}  // namespace RoverCan2::Drivers

#endif  // DRIVER_LINUX_HPP
