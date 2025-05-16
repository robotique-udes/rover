// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rover_msgs:msg/CanDeviceStatus.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "rover_msgs/msg/detail/can_device_status__struct.h"
#include "rover_msgs/msg/detail/can_device_status__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rover_msgs__msg__can_device_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("rover_msgs.msg._can_device_status.CanDeviceStatus", full_classname_dest, 49) == 0);
  }
  rover_msgs__msg__CanDeviceStatus * ros_message = _ros_message;
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // error_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // watchdog_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "watchdog_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->watchdog_ok = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rover_msgs__msg__can_device_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CanDeviceStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rover_msgs.msg._can_device_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CanDeviceStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rover_msgs__msg__CanDeviceStatus * ros_message = (rover_msgs__msg__CanDeviceStatus *)raw_ros_message;
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->error_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // watchdog_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->watchdog_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "watchdog_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
