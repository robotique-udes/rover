#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include "helpers/log.h"
#include "helpers/microROS_manager.h"

#define NAME_NS "/template_ESP32"
#define NAME_NODE "simple_example"

// =============================================================================
//  This project can be used as a starting template for ESP32 microROS project
//
//  Logging:
//      Since microROS is talking over serial, you can't directly use 
//      Serial.print to print. Instead use the LOG() macro ex:
//          LOG(INFO, "some_text %i", some_int);
//
//      To see the log in a terminal, run this cmd in a terminal:
//          ros2 launch rover_helper terminal_logger.py
//
//      If the node is connected with a micro_ros_agent, it will output in the 
//      terminal. If, for some reasons, your node isn't connecting, the output 
//      will be printed in a terminal at 115200 baud.
//
//      The logging level setting can be changed in the platformio.ini file by
//      changing the following entry: '-D LOGGER_LOWEST_LEVEL=10' to the level 
//      you want (10, 20, 30, 40, 50)
//
//  Developpement guidelines:
//      Write all function that can be reused in other projects in the
//      "lib_rover" library located in rover/rover_micro_ros_projects/lib_rover
//      the library is included in the platformio.ini file
//
//  MicroROSManagerCustom:
//      This class is a helper that handles all necessary connection with a
//      micro_ros_agent for you.
//
// =============================================================================

// Function forward declarations
bool createEntities();
void destroyEntities();
void cbTimer(rcl_timer_t *timer, int64_t last_call_time);
void cbSubscriber(const void *msg_);

// Global objects
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t pub;
rcl_subscription_t sub;
int32_t counter;

// The setup is used as main function to limit the use of global variables,
// the loop function isn't used, instead it loops inside here
void setup()
{
    // Open USB serial port and give it to micro_ros for communication with 
    // micro_ros_agent. 
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // This object handles all connection with the micro_ros_agent, you must 
    // pass two function pointer, createEntities which is called when connecting
    // and destroyEntitities which is called after a deconnection.
    MicroROSManagerCustom rosManager(createEntities, destroyEntities, true);
    rosManager.init();

    for (EVER)
    {
        // digitalWrite(LED_BUILTIN, HIGH);
        // This check if any callback are ready (new msg received or a done 
        // timer). It must be call quite often, each loop is a rule of thumb. 
        // This is also where MicroROSManagerCustom check connection state and 
        // acts accordingly
        rosManager.spinSome(&executor, 0UL);
    }
}

// Based on micro_ros_platformio example, this is a ok generic micro ros
// initialisation, modify it according to your needs.
bool createEntities()
{
    // (required)
    allocator = rcl_get_default_allocator();

    // create init_options (required)
    RCLC_RET_ON_ERR(rclc_support_init(&support, 0, NULL, &allocator));

    // create node (required)
    RCLC_RET_ON_ERR(rclc_node_init_default(&node, NAME_NODE, NAME_NS, &support));

    // create logger (optionnal)
    // This creates a publisher for logging to /debug/rovus_debug, without 
    // this line you can't the the terminal_logger.
    Logger.createLogger(&node, NAME_NODE, NAME_NS);

    // create publisher (optional)
    RCLC_RET_ON_ERR(rclc_publisher_init_default(&pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "pub_topic_name"));

    // create timer (optional)
    // The passed function is called each time each time the timer is ready at
    // the "rosManager.spinSome(&executor, 0UL);" call")
    RCLC_RET_ON_ERR(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), cbTimer));

    // create subscriber (optional)
    RCLC_RET_ON_ERR(rclc_subscription_init_default(&sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "sub_topic_name"))

    // create executor (required)
    executor = rclc_executor_get_zero_initialized_executor();

    // Set the number of handle (here 2) is the total number of subscriptions,
    // timers, services, clients and guard conditions. Do not include the number
    // of nodes and publishers. (required)
    RCLC_RET_ON_ERR(rclc_executor_init(&executor, &support.context, 2, &allocator));

    // Add this line for each timer (links the timer with the executor)
    RCLC_RET_ON_ERR(rclc_executor_add_timer(&executor, &timer));

    // Add these lines for each subscriber (links the sub with the executor), 
    // You also need to create a temporary msg of the sub msg type and passed 
    // the pointer to the executor for memory allocation
    std_msgs__msg__Int32 msg;
    RCLC_RET_ON_ERR(rclc_executor_add_subscription(&executor, &sub, &msg, &cbSubscriber, ON_NEW_DATA));

    return true;
}

void destroyEntities()
{
    // necessary
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // "Fini" each element created inside the createEntities
    Logger.destroyLogger(&node);
    REMOVE_WARN_UNUSED(rcl_publisher_fini(&pub, &node));
    REMOVE_WARN_UNUSED(rcl_timer_fini(&timer));
    REMOVE_WARN_UNUSED(rcl_subscription_fini(&sub, &node));
    REMOVE_WARN_UNUSED(rclc_executor_fini(&executor));
    REMOVE_WARN_UNUSED(rcl_node_fini(&node));
    REMOVE_WARN_UNUSED(rclc_support_fini(&support));
}

// This function is called each at the set timer frequency
void cbTimer(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL)
    {
        std_msgs__msg__Int32 msg;
        msg.data = counter++;
        REMOVE_WARN_UNUSED(rcl_publish(&pub, &msg, NULL));
        LOG(INFO, "Sent %i", msg.data);
    }
}

void cbSubscriber(const void *msg_)
{
    const std_msgs__msg__Int32 *msg = (std_msgs__msg__Int32*)msg_; 

    LOG(INFO, "Received: %i", msg->data);
}
