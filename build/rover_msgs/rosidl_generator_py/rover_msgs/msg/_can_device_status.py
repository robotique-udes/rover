# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:msg/CanDeviceStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CanDeviceStatus(type):
    """Metaclass of message 'CanDeviceStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STATUS_OK': 0,
        'STATUS_WARNING': 1,
        'STATUS_ERROR': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rover_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rover_msgs.msg.CanDeviceStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__can_device_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__can_device_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__can_device_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__can_device_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__can_device_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'STATUS_OK': cls.__constants['STATUS_OK'],
            'STATUS_WARNING': cls.__constants['STATUS_WARNING'],
            'STATUS_ERROR': cls.__constants['STATUS_ERROR'],
        }

    @property
    def STATUS_OK(self):
        """Message constant 'STATUS_OK'."""
        return Metaclass_CanDeviceStatus.__constants['STATUS_OK']

    @property
    def STATUS_WARNING(self):
        """Message constant 'STATUS_WARNING'."""
        return Metaclass_CanDeviceStatus.__constants['STATUS_WARNING']

    @property
    def STATUS_ERROR(self):
        """Message constant 'STATUS_ERROR'."""
        return Metaclass_CanDeviceStatus.__constants['STATUS_ERROR']


class CanDeviceStatus(metaclass=Metaclass_CanDeviceStatus):
    """
    Message class 'CanDeviceStatus'.

    Constants:
      STATUS_OK
      STATUS_WARNING
      STATUS_ERROR
    """

    __slots__ = [
        '_id',
        '_error_state',
        '_watchdog_ok',
    ]

    _fields_and_field_types = {
        'id': 'uint16',
        'error_state': 'uint8',
        'watchdog_ok': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        self.error_state = kwargs.get('error_state', int())
        self.watchdog_ok = kwargs.get('watchdog_ok', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.id != other.id:
            return False
        if self.error_state != other.error_state:
            return False
        if self.watchdog_ok != other.watchdog_ok:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'id' field must be an unsigned integer in [0, 65535]"
        self._id = value

    @builtins.property
    def error_state(self):
        """Message field 'error_state'."""
        return self._error_state

    @error_state.setter
    def error_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'error_state' field must be an unsigned integer in [0, 255]"
        self._error_state = value

    @builtins.property
    def watchdog_ok(self):
        """Message field 'watchdog_ok'."""
        return self._watchdog_ok

    @watchdog_ok.setter
    def watchdog_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'watchdog_ok' field must be of type 'bool'"
        self._watchdog_ok = value
