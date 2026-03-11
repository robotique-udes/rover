#ifndef ROVER_LIB2_ACTUATORS_ACTUATOR_HPP
#define ROVER_LIB2_ACTUATORS_ACTUATOR_HPP

#include <rover_lib2/rover_object.hpp>
#include <rover_lib2/helpers/assert.hpp>

#include <concepts>
#include <optional>

namespace Actuators
{
    template<typename ImplT>
    concept Actuator = RoverObject<ImplT> && requires(ImplT impl_) {
        // clang-format off

        { impl_.setPosition(float{} /*pos_*/) } -> std::same_as<void>;

        { std::as_const(impl_).getPosition() } -> std::same_as<float>;

        { impl_.setSpeed(float{} /*speed_*/) } -> std::same_as<void>;

        { std::as_const(impl_).getSpeed() } -> std::same_as<float>;

        { impl_.setMaxSpeed(float{} /*max_speed_*/) } -> std::same_as<void>;

        { impl_.calib(float{} /*offset_*/) } -> std::same_as<void>;

        /**
         * @brief Sets the joint limits of the actuators, using std::nullopt will reset limits
         *
         */
        { impl_.setJointLimit(std::optional<float>{} /*min_*/, std::optional<float>{} /*max_*/) } -> std::same_as<void>;

        // clang-format on
    };

    class None
    {
      public:
        void init()
        {
            ASSERT_MSG("Interface");
        }

        void update()
        {
            ASSERT_MSG("Interface");
        }

        void setPosition(float /*pos_*/)
        {
            ASSERT_MSG("Interface");
        }

        float getPosition()
        {
            ASSERT_MSG("Interface");
            return 0.0F;
        }

        void setSpeed(float /*speed_*/)
        {
            ASSERT_MSG("Interface");
        }

        float getSpeed()
        {
            ASSERT_MSG("Interface");
            return 0.0F;
        }

        void setMaxSpeed(float /*max_speed_*/)
        {
            ASSERT_MSG("Interface");
        }

        /**
         * @brief Sets the joint limits of the actuators, using std::nullopt will reset limits
         *
         */
        void setJointLimit(std::optional<float> /*min_*/, std::optional<float> /*max_*/)
        {
            ASSERT_MSG("Interface");
        }
    };

}  // namespace Actuators

#endif  // ROVER_LIB2_ACTUATORS_ACTUATOR_HPP
