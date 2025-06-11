#ifndef ROVER_LIB2_IO_DIGITAL_IO_HPP
#define ROVER_LIB2_IO_DIGITAL_IO_HPP

#include <cstdint>

namespace IO
{
    enum class eIOState : uint32_t
    {
        LOW_ = 0U,
        HIGH_ = 1U,
    };

}  // namespace IO

#endif  // ROVER_LIB2_IO_DIGITAL_IO_HPP
