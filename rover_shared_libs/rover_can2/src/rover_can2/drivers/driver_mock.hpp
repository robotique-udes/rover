#ifndef ROVER_CAN2_DRIVERS_DRIVER_MOCK_HPP
#define ROVER_CAN2_DRIVERS_DRIVER_MOCK_HPP

#include "rover_can2/drivers/driver_base.hpp"
#include "rover_lib2/helpers/circular_buffer.hpp"

namespace RoverCan2::Drivers
{
    /**
     * @brief Used for tests only
     *
     */
    class DriverMock : public DriverBase<DriverMock>
    {
      public:
        void _init(void)
        {
            isInited = true;
        }

        void _update(void)
        {
            hasUpdated = true;
            // Must be done manually in tests
        }

        size_t _getAvailableMessagesNb(void) const
        {
            return newMsgsBuffer.size();
        }

        std::optional<CanMsg> _getMsg(void)
        {
            return newMsgsBuffer.getValue();
        }

        bool sendMsg(const CanMsg& msg_)
        {
            msgSentBuffer.addValue(msg_);
            return true;
        }

        CircularBuffer<CanMsg, 100> msgSentBuffer;
        CircularBuffer<CanMsg, 10> newMsgsBuffer;
        bool isInited = false;
        bool hasUpdated = false;
    };
}  // namespace RoverCan2::Drivers
#endif  // ROVER_CAN2_DRIVERS_DRIVER_MOCK_HPP
