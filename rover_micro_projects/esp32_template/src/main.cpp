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

// TODO Make helpers + clean up + write template with comments

bool createEntities();
void destroyEntities();
void cbTimer(rcl_timer_t *timer, int64_t last_call_time);

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t pub;

int32_t counter;

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    MicroROSManagerCustom rosManager(createEntities, destroyEntities);

    for (EVER)
    {
        rosManager.spinSome(&executor, 0UL);
    }
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    // create init_options
    RCLC_RET_ON_ERR(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCLC_RET_ON_ERR(rclc_node_init_default(&node, NAME_NODE, "", &support));

    // create logger (optionnal)
    Logger.createLogger(&node, NAME_NODE);

    // create publisher (optional)
    std_msgs__msg__Int32 msg;
    RCLC_RET_ON_ERR(rclc_publisher_init_default(&pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "topicName"));

    // create timer (optional)
    RCLC_RET_ON_ERR(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), cbTimer));

    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCLC_RET_ON_ERR(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCLC_RET_ON_ERR(rclc_executor_add_timer(&executor, &timer));

    return true;
}

void destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    Logger.destroyLogger(&node);
    REMOVE_WARN_UNUSED(rcl_publisher_fini(&pub, &node));
    REMOVE_WARN_UNUSED(rcl_timer_fini(&timer));
    rclc_executor_fini(&executor);
    REMOVE_WARN_UNUSED(rcl_node_fini(&node));
    rclc_support_fini(&support);
}

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
