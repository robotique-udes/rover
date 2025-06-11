#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP

#include <rover_lib2/rover_object.hpp>

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

    void setJointLimit(float min_, float max_)
    {
        static_cast<Impl_T*>(this)->_setJointLimit(min_, max_);
    }

    void setReversed(bool reversed_)
    {
        static_cast<Impl_T*>(this)->_setReversed(reversed_);
    }
};

#endif  // ACTUATOR_HPP
