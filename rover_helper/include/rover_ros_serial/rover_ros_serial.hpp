#ifndef __ROVER_ROS_SERIAL_HPP__
#define __ROVER_ROS_SERIAL_HPP__

#if defined(ESP32)
#include "Arduino.h"
#include "helpers/timer.h"
#include "helpers/macros.h"
#include "helpers/log.h"

#endif // defined(ESP32)

#include "rover_ros_serial/base_objects.hpp"
#include "rover_ros_serial/msg.hpp"
#include "logger.hpp"

#if defined(ESP32)
#include "heartbeat.hpp"
#include "node.hpp"

#endif // defined(ESP32)

#endif // __ROVER_ROS_SERIAL_HPP__
