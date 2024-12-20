# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:msg/Joy.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'joy_data'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Joy(type):
    """Metaclass of message 'Joy'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'JOYSTICK_LEFT_FRONT': 0,
        'JOYSTICK_LEFT_SIDE': 1,
        'JOYSTICK_LEFT_PUSH': 2,
        'JOYSTICK_RIGHT_FRONT': 3,
        'JOYSTICK_RIGHT_SIDE': 4,
        'JOYSTICK_RIGHT_PUSH': 5,
        'CROSS_UP': 6,
        'CROSS_DOWN': 7,
        'CROSS_LEFT': 8,
        'CROSS_RIGHT': 9,
        'L1': 10,
        'L2': 11,
        'R1': 12,
        'R2': 13,
        'A': 14,
        'B': 15,
        'X': 16,
        'Y': 17,
        'EXT0': 18,
        'EXT1': 19,
        'EXT2': 20,
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
                'rover_msgs.msg.Joy')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__joy
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__joy
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__joy
            cls._TYPE_SUPPORT = module.type_support_msg__msg__joy
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__joy

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'JOYSTICK_LEFT_FRONT': cls.__constants['JOYSTICK_LEFT_FRONT'],
            'JOYSTICK_LEFT_SIDE': cls.__constants['JOYSTICK_LEFT_SIDE'],
            'JOYSTICK_LEFT_PUSH': cls.__constants['JOYSTICK_LEFT_PUSH'],
            'JOYSTICK_RIGHT_FRONT': cls.__constants['JOYSTICK_RIGHT_FRONT'],
            'JOYSTICK_RIGHT_SIDE': cls.__constants['JOYSTICK_RIGHT_SIDE'],
            'JOYSTICK_RIGHT_PUSH': cls.__constants['JOYSTICK_RIGHT_PUSH'],
            'CROSS_UP': cls.__constants['CROSS_UP'],
            'CROSS_DOWN': cls.__constants['CROSS_DOWN'],
            'CROSS_LEFT': cls.__constants['CROSS_LEFT'],
            'CROSS_RIGHT': cls.__constants['CROSS_RIGHT'],
            'L1': cls.__constants['L1'],
            'L2': cls.__constants['L2'],
            'R1': cls.__constants['R1'],
            'R2': cls.__constants['R2'],
            'A': cls.__constants['A'],
            'B': cls.__constants['B'],
            'X': cls.__constants['X'],
            'Y': cls.__constants['Y'],
            'EXT0': cls.__constants['EXT0'],
            'EXT1': cls.__constants['EXT1'],
            'EXT2': cls.__constants['EXT2'],
        }

    @property
    def JOYSTICK_LEFT_FRONT(self):
        """Message constant 'JOYSTICK_LEFT_FRONT'."""
        return Metaclass_Joy.__constants['JOYSTICK_LEFT_FRONT']

    @property
    def JOYSTICK_LEFT_SIDE(self):
        """Message constant 'JOYSTICK_LEFT_SIDE'."""
        return Metaclass_Joy.__constants['JOYSTICK_LEFT_SIDE']

    @property
    def JOYSTICK_LEFT_PUSH(self):
        """Message constant 'JOYSTICK_LEFT_PUSH'."""
        return Metaclass_Joy.__constants['JOYSTICK_LEFT_PUSH']

    @property
    def JOYSTICK_RIGHT_FRONT(self):
        """Message constant 'JOYSTICK_RIGHT_FRONT'."""
        return Metaclass_Joy.__constants['JOYSTICK_RIGHT_FRONT']

    @property
    def JOYSTICK_RIGHT_SIDE(self):
        """Message constant 'JOYSTICK_RIGHT_SIDE'."""
        return Metaclass_Joy.__constants['JOYSTICK_RIGHT_SIDE']

    @property
    def JOYSTICK_RIGHT_PUSH(self):
        """Message constant 'JOYSTICK_RIGHT_PUSH'."""
        return Metaclass_Joy.__constants['JOYSTICK_RIGHT_PUSH']

    @property
    def CROSS_UP(self):
        """Message constant 'CROSS_UP'."""
        return Metaclass_Joy.__constants['CROSS_UP']

    @property
    def CROSS_DOWN(self):
        """Message constant 'CROSS_DOWN'."""
        return Metaclass_Joy.__constants['CROSS_DOWN']

    @property
    def CROSS_LEFT(self):
        """Message constant 'CROSS_LEFT'."""
        return Metaclass_Joy.__constants['CROSS_LEFT']

    @property
    def CROSS_RIGHT(self):
        """Message constant 'CROSS_RIGHT'."""
        return Metaclass_Joy.__constants['CROSS_RIGHT']

    @property
    def L1(self):
        """Message constant 'L1'."""
        return Metaclass_Joy.__constants['L1']

    @property
    def L2(self):
        """Message constant 'L2'."""
        return Metaclass_Joy.__constants['L2']

    @property
    def R1(self):
        """Message constant 'R1'."""
        return Metaclass_Joy.__constants['R1']

    @property
    def R2(self):
        """Message constant 'R2'."""
        return Metaclass_Joy.__constants['R2']

    @property
    def A(self):
        """Message constant 'A'."""
        return Metaclass_Joy.__constants['A']

    @property
    def B(self):
        """Message constant 'B'."""
        return Metaclass_Joy.__constants['B']

    @property
    def X(self):
        """Message constant 'X'."""
        return Metaclass_Joy.__constants['X']

    @property
    def Y(self):
        """Message constant 'Y'."""
        return Metaclass_Joy.__constants['Y']

    @property
    def EXT0(self):
        """Message constant 'EXT0'."""
        return Metaclass_Joy.__constants['EXT0']

    @property
    def EXT1(self):
        """Message constant 'EXT1'."""
        return Metaclass_Joy.__constants['EXT1']

    @property
    def EXT2(self):
        """Message constant 'EXT2'."""
        return Metaclass_Joy.__constants['EXT2']


class Joy(metaclass=Metaclass_Joy):
    """
    Message class 'Joy'.

    Constants:
      JOYSTICK_LEFT_FRONT
      JOYSTICK_LEFT_SIDE
      JOYSTICK_LEFT_PUSH
      JOYSTICK_RIGHT_FRONT
      JOYSTICK_RIGHT_SIDE
      JOYSTICK_RIGHT_PUSH
      CROSS_UP
      CROSS_DOWN
      CROSS_LEFT
      CROSS_RIGHT
      L1
      L2
      R1
      R2
      A
      B
      X
      Y
      EXT0
      EXT1
      EXT2
    """

    __slots__ = [
        '_joy_data',
    ]

    _fields_and_field_types = {
        'joy_data': 'float[20]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 20),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'joy_data' not in kwargs:
            self.joy_data = numpy.zeros(20, dtype=numpy.float32)
        else:
            self.joy_data = numpy.array(kwargs.get('joy_data'), dtype=numpy.float32)
            assert self.joy_data.shape == (20, )

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
        if all(self.joy_data != other.joy_data):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def joy_data(self):
        """Message field 'joy_data'."""
        return self._joy_data

    @joy_data.setter
    def joy_data(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'joy_data' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 20, \
                "The 'joy_data' numpy.ndarray() must have a size of 20"
            self._joy_data = value
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
                 len(value) == 20 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'joy_data' field must be a set or sequence with length 20 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._joy_data = numpy.array(value, dtype=numpy.float32)
