# Micro controllers projects

## Creating a new project

## ESP32 Setup
Since were running ROS2, all microcontrollers will run [microROS](https://micro.ros.org/) and all developpement will be in platformIO. This [tutorial](https://www.youtube.com/watch?v=Nf7HP9y6Ovo&t=435s) is a great first step at learning microROS.
### General guidelines
 - Use ESP32 for all microcontrollers projects
 - Develop in VSCode with platformIO
 - DON'T use spaces in platformIO project names (this will cause issues with micro_ros_platformio (see [#119](https://github.com/micro-ROS/micro_ros_platformio/issues/119)) 
 - Use serial transport
 - Run your serial ports at 115200 baud
 - Add your project to rover_micro_projects
 - Define all classes/functions/macro/etc that can be reused in the rover_micro_projects/lib_rover/ to limit copy/paste between projects
 - Limit dynamic allocation and follow embedded coding guidelines as much as you can

### platformio.ini
This file contains your [platformio project configuration](https://docs.platformio.org/en/latest/projectconf/index.html). For rover project, your configuration will need to include at least the following elements:
```INI
[env:nodemcu-32s]
; PLatformIO doesn't directly support our cheap ESP32, this git repo fix this
platform = https://github.com/Jason2866/platform-espressif32.git
framework = arduino
board = nodemcu-32s

monitor_speed = 115200 ; monitor baud rate
monitor_raw = true ; allow colors inside terminal

; microros config
board_microros_transport = serial 
board_microros_distro = humble

; librairies
lib_deps = 
    rover_lib=symlink://../lib_rover
    https://github.com/micro-ROS/micro_ros_platformio.git

; Act as #define CONSTANT 
build_flags = 
    ; When defined, LOG() macro are compilated, otherwise it's skip for performance
	'-D VERBOSE'
    ; Only shows higher logs levels. Levels are: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
    '-D LOGGER_LOWEST_LEVEL=20'
    ; Do not change this is used for the debug_printer.cpp(terminal_logger.py)
    '-D NAME_LOG_TOPIC="/debug/rovus_debug"'
```

The ESP32 we currently own are cheap dev boards and they don't work out of the box with platformio. To fix this modify this line in your platformio.ini file:
```INI
platform = https://github.com/Jason2866/platform-espressif32.git
```

### Buying guide
When buying ESP32 or any microcontrollers, make sure it comes with a unique USB ID. The FTDI (usb to UART) IC (chip) on the board should take care of this but some cheap chips all have the same id. This make identifying which board is which quite hard and has caused problem in previous competitions.
- Safe bets:
  - CH9102X
- To avoid:
  - CP2102 
  - CH430

Here is an example of the FTDI chip:

![test](../doc/img/esp32-ftdi.gif)

# ESP32 and micro controller ROS developpement:
See [rover_micro_projects/readme.md] 
