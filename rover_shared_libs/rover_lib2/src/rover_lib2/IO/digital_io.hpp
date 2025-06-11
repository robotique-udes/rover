#ifndef DIGITAL_IO_HPP
#define DIGITAL_IO_HPP

#include <cstdint>

namespace IO
{
    enum class eIOState : uint32_t
    {
        LOW_ = 0U,
        HIGH_ = 1U,
    };

}  // namespace IO

#endif  // DIGITAL_IO_HPP
