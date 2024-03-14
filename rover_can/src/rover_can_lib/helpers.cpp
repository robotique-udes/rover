#include "rover_can_lib/helpers.hpp"

namespace RoverCanLib::Helpers
{
    // Return a msg with ID 0x00 which should be never be considered by any
    // nodes, it's used internally to say that theres no more message in the rx
    // queue
#if defined(ESP32)
    twai_message_t getErrorIdMsg(void)
    {
        twai_message_t msg;
        msg.identifier = 0u;

        return msg;
    }
#endif // defined(ESP32)
}
