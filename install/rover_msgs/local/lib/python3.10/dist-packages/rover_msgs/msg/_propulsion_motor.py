# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:msg/PropulsionMotor.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'target_speed'
# Member 'current_speed'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PropulsionMotor(type):
    """Metaclass of message 'PropulsionMotor'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FRONT_LEFT': 0,
        'FRONT_RIGHT': 1,
        'REAR_LEFT': 2,
        'REAR_RIGHT': 3,
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
                'rover_msgs.msg.PropulsionMotor')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__propulsion_motor
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__propulsion_motor
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__propulsion_motor
            cls._TYPE_SUPPORT = module.type_support_msg__msg__propulsion_motor
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__propulsion_motor

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FRONT_LEFT': cls.__constants['FRONT_LEFT'],
            'FRONT_RIGHT': cls.__constants['FRONT_RIGHT'],
            'REAR_LEFT': cls.__constants['REAR_LEFT'],
            'REAR_RIGHT': cls.__constants['REAR_RIGHT'],
        }

    @property
    def FRONT_LEFT(self):
        """Message constant 'FRONT_LEFT'."""
        return Metaclass_PropulsionMotor.__constants['FRONT_LEFT']

    @property
    def FRONT_RIGHT(self):
        """Message constant 'FRONT_RIGHT'."""
        return Metaclass_PropulsionMotor.__constants['FRONT_RIGHT']

    @property
    def REAR_LEFT(self):
        """Message constant 'REAR_LEFT'."""
        return Metaclass_PropulsionMotor.__constants['REAR_LEFT']

    @property
    def REAR_RIGHT(self):
        """Message constant 'REAR_RIGHT'."""
        return Metaclass_PropulsionMotor.__constants['REAR_RIGHT']


class PropulsionMotor(metaclass=Metaclass_PropulsionMotor):
    """
    Message class 'PropulsionMotor'.

    Constants:
      FRONT_LEFT
      FRONT_RIGHT
      REAR_LEFT
      REAR_RIGHT
    """

    __slots__ = [
        '_enable',
        '_target_speed',
        '_current_speed',
        '_close_loop',
    ]

    _fields_and_field_types = {
        'enable': 'boolean[4]',
        'target_speed': 'float[4]',
        'current_speed': 'float[4]',
        'close_loop': 'boolean[4]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('boolean'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('boolean'), 4),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.enable = kwargs.get(
            'enable',
            [bool() for x in range(4)]
        )
        if 'target_speed' not in kwargs:
            self.target_speed = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.target_speed = numpy.array(kwargs.get('target_speed'), dtype=numpy.float32)
            assert self.target_speed.shape == (4, )
        if 'current_speed' not in kwargs:
            self.current_speed = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.current_speed = numpy.array(kwargs.get('current_speed'), dtype=numpy.float32)
            assert self.current_speed.shape == (4, )
        self.close_loop = kwargs.get(
            'close_loop',
            [bool() for x in range(4)]
        )

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
        if self.enable != other.enable:
            return False
        if all(self.target_speed != other.target_speed):
            return False
        if all(self.current_speed != other.current_speed):
            return False
        if self.close_loop != other.close_loop:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def enable(self):
        """Message field 'enable'."""
        return self._enable

    @enable.setter
    def enable(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'enable' field must be a set or sequence with length 4 and each value of type 'bool'"
        self._enable = value

    @builtins.property
    def target_speed(self):
        """Message field 'target_speed'."""
        return self._target_speed

    @target_speed.setter
    def target_speed(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'target_speed' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 4, \
                "The 'target_speed' numpy.ndarray() must have a size of 4"
            self._target_speed = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'target_speed' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._target_speed = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def current_speed(self):
        """Message field 'current_speed'."""
        return self._current_speed

    @current_speed.setter
    def current_speed(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'current_speed' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 4, \
                "The 'current_speed' numpy.ndarray() must have a size of 4"
            self._current_speed = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'current_speed' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._current_speed = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def close_loop(self):
        """Message field 'close_loop'."""
        return self._close_loop

    @close_loop.setter
    def close_loop(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'close_loop' field must be a set or sequence with length 4 and each value of type 'bool'"
        self._close_loop = value
