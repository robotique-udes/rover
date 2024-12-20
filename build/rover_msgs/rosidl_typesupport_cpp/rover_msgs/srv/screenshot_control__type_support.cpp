// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from rover_msgs:srv/ScreenshotControl.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rover_msgs/srv/detail/screenshot_control__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace rover_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ScreenshotControl_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ScreenshotControl_Request_type_support_ids_t;

static const _ScreenshotControl_Request_type_support_ids_t _ScreenshotControl_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ScreenshotControl_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ScreenshotControl_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ScreenshotControl_Request_type_support_symbol_names_t _ScreenshotControl_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rover_msgs, srv, ScreenshotControl_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rover_msgs, srv, ScreenshotControl_Request)),
  }
};

typedef struct _ScreenshotControl_Request_type_support_data_t
{
  void * data[2];
} _ScreenshotControl_Request_type_support_data_t;

static _ScreenshotControl_Request_type_support_data_t _ScreenshotControl_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ScreenshotControl_Request_message_typesupport_map = {
  2,
  "rover_msgs",
  &_ScreenshotControl_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ScreenshotControl_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ScreenshotControl_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ScreenshotControl_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ScreenshotControl_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace rover_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rover_msgs::srv::ScreenshotControl_Request>()
{
  return &::rover_msgs::srv::rosidl_typesupport_cpp::ScreenshotControl_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rover_msgs, srv, ScreenshotControl_Request)() {
  return get_message_type_support_handle<rover_msgs::srv::ScreenshotControl_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rover_msgs/srv/detail/screenshot_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rover_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ScreenshotControl_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ScreenshotControl_Response_type_support_ids_t;

static const _ScreenshotControl_Response_type_support_ids_t _ScreenshotControl_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ScreenshotControl_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ScreenshotControl_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ScreenshotControl_Response_type_support_symbol_names_t _ScreenshotControl_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rover_msgs, srv, ScreenshotControl_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rover_msgs, srv, ScreenshotControl_Response)),
  }
};

typedef struct _ScreenshotControl_Response_type_support_data_t
{
  void * data[2];
} _ScreenshotControl_Response_type_support_data_t;

static _ScreenshotControl_Response_type_support_data_t _ScreenshotControl_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ScreenshotControl_Response_message_typesupport_map = {
  2,
  "rover_msgs",
  &_ScreenshotControl_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ScreenshotControl_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ScreenshotControl_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ScreenshotControl_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ScreenshotControl_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace rover_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rover_msgs::srv::ScreenshotControl_Response>()
{
  return &::rover_msgs::srv::rosidl_typesupport_cpp::ScreenshotControl_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rover_msgs, srv, ScreenshotControl_Response)() {
  return get_message_type_support_handle<rover_msgs::srv::ScreenshotControl_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rover_msgs/srv/detail/screenshot_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rover_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ScreenshotControl_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ScreenshotControl_type_support_ids_t;

static const _ScreenshotControl_type_support_ids_t _ScreenshotControl_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ScreenshotControl_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ScreenshotControl_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ScreenshotControl_type_support_symbol_names_t _ScreenshotControl_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rover_msgs, srv, ScreenshotControl)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rover_msgs, srv, ScreenshotControl)),
  }
};

typedef struct _ScreenshotControl_type_support_data_t
{
  void * data[2];
} _ScreenshotControl_type_support_data_t;

static _ScreenshotControl_type_support_data_t _ScreenshotControl_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ScreenshotControl_service_typesupport_map = {
  2,
  "rover_msgs",
  &_ScreenshotControl_service_typesupport_ids.typesupport_identifier[0],
  &_ScreenshotControl_service_typesupport_symbol_names.symbol_name[0],
  &_ScreenshotControl_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ScreenshotControl_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ScreenshotControl_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace rover_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<rover_msgs::srv::ScreenshotControl>()
{
  return &::rover_msgs::srv::rosidl_typesupport_cpp::ScreenshotControl_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp
