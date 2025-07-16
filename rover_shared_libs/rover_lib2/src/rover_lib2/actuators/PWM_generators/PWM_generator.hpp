#ifndef ROVER_LIB2_ACTUATORS_PWM_GENERATORS_PWM_GENERATOR_HPP
#define ROVER_LIB2_ACTUATORS_PWM_GENERATORS_PWM_GENERATOR_HPP

#include "rover_lib2/rover_object.hpp"

namespace PWMGenerators
{

    template<typename ImplT>
    concept PWMGenerator = RoverObject<ImplT> && requires(ImplT impl_)
    {
        // clang-format off
        { impl_.setDutyCycle(float{} /* duty_ */) } -> std::same_as<void>;

        { std::as_const(impl_).getDutyCycle() } -> std::same_as<float> ;

        { impl_.setFrequency(float{} /* freq_ */) } -> std::same_as<void>;

        { std::as_const(impl_).getFrequency() } -> std::same_as<float> ;

        { impl_.setEnabled(bool{} /* enabled_ */) } -> std::same_as<void> ;

        { std::as_const(impl_).isEnabled() } -> std::same_as<bool> ;
        // clang-format on
    };

}  // namespace PWMGenerators

#endif  // ROVER_LIB2_ACTUATORS_PWM_GENERATORS_PWM_GENERATOR_HPP
