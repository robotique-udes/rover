#ifndef __ROBOT_CONTROLLER_HPP__
#define __ROBOT_CONTROLLER_HPP__

#include "rovus_lib/timer.hpp"
#include "rovus_lib/macros.h"

#include <stdint.h>
#include <vector>
#include <array>

template <size_t N>
class RobotController
{
  public:
    enum class eControlMode : uint8_t
    {
        CARTESIAN,
        JOINT,
        NONE
    };

  protected:
    RobotController(RobotController::eControlMode controlMode_ = RobotController::eControlMode::NONE)
        : _controlMode(controlMode_)
    {
    }

  public:
    virtual ~RobotController(void) = default;
    virtual void setCmd(std::array<size_t, N>) = 0;
    virtual void setMode(eControlMode mode_) = 0;
    eControlMode getMode(void) const
    {
        return _controlMode;
    }

  protected:
    eControlMode _controlMode = RobotController::eControlMode::NONE;
    static constexpr size_t _controlledElements = N;
};

#endif
