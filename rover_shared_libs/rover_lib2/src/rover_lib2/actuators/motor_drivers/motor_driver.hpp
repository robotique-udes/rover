#ifndef ROVER_LIB2_ACTUATORS_MOTOR_DRIVERS_MOTOR_DRIVER_HPP
#define ROVER_LIB2_ACTUATORS_MOTOR_DRIVERS_MOTOR_DRIVER_HPP

#include "rover_lib2/rover_object.hpp"

#include <cstdint>

class MotorDriverT
{
  public:
    enum class eBrakeMode : uint8_t
    {
        BRAKE,
        COAST
    };

  protected:
    MotorDriverT() = default;
};

template<typename Impl_T>
class MotorDriver : public MotorDriverT,
                    public RoverObject<MotorDriver<Impl_T>>

{
  private:
    friend Impl_T;
    MotorDriver() = default;

  public:
    void _init(void)
    {
        static_cast<Impl_T*>(this)->__init();
    }

    void _update(void)
    {
        static_cast<Impl_T*>(this)->__update();
    }

    // Range is [-100; 100]
    void setCmd(float cmd_)
    {
        static_cast<Impl_T*>(this)->_setCmd(cmd_);
    }

    // Range is [-100; 100]
    float getCmd(void)
    {
        return static_cast<Impl_T*>(this)->_getCmd();
    }

    void setEnabled(bool on_)
    {
        static_cast<Impl_T*>(this)->_setEnabled(on_);
    }

    bool isEnabled(void)
    {
        return static_cast<Impl_T*>(this)->_isEnabled();
    }

    void setReversed(bool reversed_)
    {
        static_cast<Impl_T*>(this)->_setReversed(reversed_);
    }

    bool isReversed(void)
    {
        return static_cast<Impl_T*>(this)->_isReversed();
    }

    void setBrakeMode(eBrakeMode mode_)
    {
        static_cast<Impl_T*>(this)->_setBrakeMode(mode_);
    }

    eBrakeMode getBrakeMode(void)
    {
        return static_cast<Impl_T*>(this)->_getBrakeMode();
    }
};

#endif  // ROVER_LIB2_ACTUATORS_MOTOR_DRIVERS_MOTOR_DRIVER_HPP
