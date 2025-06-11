#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <rover_lib2/rover_object.hpp>

template<typename Impl_T>
class Encoder : public RoverObject<Encoder<Impl_T>>
{
  private:
    friend Impl_T;
    Encoder() = default;

  public:
    void _init(void)
    {
        static_cast<Impl_T*>(this)->__init();
    }

    void _update(void)
    {
        static_cast<Impl_T*>(this)->__update();
    }

    bool dataIsValid(void)
    {
        return static_cast<Impl_T*>(this)->_dataIsValid();
    }

    float getPosition(void)
    {
        return static_cast<Impl_T*>(this)->_getPosition();
    }

    float getSpeed(void)
    {
        return static_cast<Impl_T*>(this)->_getSpeed();
    }

    void calib(float offset_)
    {
        static_cast<Impl_T*>(this)->_calib(offset_);
    }

    void setReversed(bool reverse_)
    {
        static_cast<Impl_T*>(this)->_setReversed(reverse_);
    }
};

#endif  // ENCODER_HPP
