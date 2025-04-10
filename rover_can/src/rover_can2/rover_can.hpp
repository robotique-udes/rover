#ifndef ROVER_CAN_HPP
#define ROVER_CAN_HPP

#include "rover_can2/manager.hpp"

#if defined(ARDUINO_ESP32S3_DEV)
#include "rover_can2/drivers/driver_esp32.hpp"
#endif  // defined(ARDUINO_ESP32S3_DEV)

#endif  // ROVER_CAN_HPP
