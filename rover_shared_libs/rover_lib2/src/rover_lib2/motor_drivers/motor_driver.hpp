#ifndef ROVER_LIB2_MOTOR_DRIVERS_MOTOR_DRIVER_HPP
#define ROVER_LIB2_MOTOR_DRIVERS_MOTOR_DRIVER_HPP

#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/helpers/assert.hpp"

#include <cstdint>
#include <concepts>

namespace MotorDrivers
{

    enum class eBrakeMode : uint8_t
    {
        BRAKE,
        COAST
    };

    template<typename ImplT>
    concept MotorDriver = RoverObject<ImplT> && requires(ImplT impl_)
    {
        // clang-format off
        { impl_.init() } -> std::same_as<void>;

        { impl_.update() } -> std::same_as<void>;

        { impl_.setCmd(float{} /*cmd_*/) } -> std::same_as<void>;

        { std::as_const(impl_).getCmd() } -> std::same_as<float> ;

        { impl_.setEnabled(bool{} /*on_*/) } -> std::same_as<void>;

        { std::as_const(impl_).isEnabled() } -> std::same_as<bool>;

        { impl_.setReversed(bool{} /*reversed_*/) } -> std::same_as<void>;

        { std::as_const(impl_).isReversed() } -> std::same_as<bool>;

        { impl_.setBrakeMode(eBrakeMode{} /*mode_*/) } -> std::same_as<void>;

        { std::as_const(impl_).getBrakeMode() } -> std::same_as<eBrakeMode>;
        // clang-format on
    };

}  // namespace MotorDrivers

#endif  // ROVER_LIB2_MOTOR_DRIVERS_MOTOR_DRIVER_HPP
