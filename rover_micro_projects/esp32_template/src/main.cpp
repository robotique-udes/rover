#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include "helpers/macros.h"
#include "helpers/connection_manager.h"

// TODO Make helpers + logging publisher + clean up + write template with comments

#define LED_PIN 13

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
int32_t counter;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL)
    {
        std_msgs__msg__Int32 msg;
        msg.data = counter++;
        REMOVE_WARN_UNUSED(rcl_publish(&publisher, &msg, NULL));
    }
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
    allocator = rcl_get_default_allocator();

    // create init_options
    RCLC_RET_ON_ERR(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCLC_RET_ON_ERR(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));

    // create publisher
    std_msgs__msg__Int32 msg;
    RCLC_RET_ON_ERR(rclc_publisher_init_default(&publisher,
                                                &node,
                                                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                                "std_msgs_msg_Int32"));

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCLC_RET_ON_ERR(rclc_timer_init_default(&timer,
                                            &support,
                                            RCL_MS_TO_NS(timer_timeout),
                                            timer_callback));

    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCLC_RET_ON_ERR(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCLC_RET_ON_ERR(rclc_executor_add_timer(&executor, &timer));

    return true;
}

void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    REMOVE_WARN_UNUSED(rcl_publisher_fini(&publisher, &node));
    REMOVE_WARN_UNUSED(rcl_timer_fini(&timer));
    rclc_executor_fini(&executor);
    REMOVE_WARN_UNUSED(rcl_node_fini(&node));
    rclc_support_fini(&support);
}

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    ConnectionManager rosManager(create_entities, destroy_entities);
    // rosManager.setTimers(500UL, 200UL);

    for (EVER)
    {
        rosManager.spinSome(&executor, 100UL);
    }
}
