#ifndef ROVER_CAN2_ROVER_CAN2_HPP
#define ROVER_CAN2_ROVER_CAN2_HPP

#include <rover_can2/manager/manager_slave.hpp>
#include <rover_can2/manager/manager_master.hpp>

#if defined(ARDUINO_ESP32S3_DEV)
#include "rover_can2/drivers/driver_esp32.hpp"
#elif defined(__linux__)
#include <rover_can2/drivers/driver_linux.hpp>
#endif  // defined(ARDUINO_ESP32S3_DEV)

#include <rover_can2/drivers/driver_mock.hpp>

#endif  // ROVER_CAN2_ROVER_CAN2_HPP
