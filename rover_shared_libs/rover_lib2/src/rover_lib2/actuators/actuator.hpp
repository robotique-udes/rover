#ifndef ROVER_LIB2_ACTUATORS_ACTUATOR_HPP
#define ROVER_LIB2_ACTUATORS_ACTUATOR_HPP

#include <rover_lib2/rover_object.hpp>

#include <optional>

namespace Actuators
{
    template<typename Impl_T>
    class Actuator : public RoverObject<Actuator<Impl_T>>
    {
      private:
        friend Impl_T;
        Actuator() = default;

      public:
        void _init(void)
        {
            static_cast<Impl_T*>(this)->__init();
        }

        void _update(void)
        {
            static_cast<Impl_T*>(this)->__update();
        }

        void setPosition(float pos_)
        {
            static_cast<Impl_T*>(this)->_setPosition(pos_);
        }

        float getPosition(void)
        {
            return static_cast<Impl_T*>(this)->_getPosition();
        }

        void setSpeed(float speed_)
        {
            static_cast<Impl_T*>(this)->_setSpeed(speed_);
        }

        float getSpeed(void)
        {
            return static_cast<Impl_T*>(this)->_getSpeed();
        }

        void setMaxSpeed(float max_speed_)
        {
            static_cast<Impl_T*>(this)->_setMaxSpeed(max_speed_);
        }

        /**
         * @brief Sets the joint limits of the actuators, using std::nullopt will reset limits
         *
         */
        void setJointLimit(std::optional<float> min_, std::optional<float> max_)
        {
            static_cast<Impl_T*>(this)->_setJointLimit(min_, max_);
        }
    };

}  // namespace Actuators

#endif  // ROVER_LIB2_ACTUATORS_ACTUATOR_HPP
