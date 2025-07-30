#ifndef ROVER_LIB2_ROVER_OBJECT_HPP
#define ROVER_LIB2_ROVER_OBJECT_HPP

#include <concepts>

template<typename Implt>
concept RoverObject = requires(Implt impl_)
{
    // clang-format off
    { impl_.init() } -> std::same_as<void>;

    { impl_.update() } -> std::same_as<void>;
    // clang-format on
};

#endif  // ROVER_LIB2_ROVER_OBJECT_HPP
