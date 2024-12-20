# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:msg/ArmMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'data'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ArmMsg(type):
    """Metaclass of message 'ArmMsg'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'JL': 0,
        'J0': 1,
        'J1': 2,
        'J2': 3,
        'GRIPPER_TILT': 4,
        'GRIPPER_ROT': 5,
        'GRIPPER_CLOSE': 6,
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
                'rover_msgs.msg.ArmMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__arm_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__arm_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__arm_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__arm_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__arm_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'JL': cls.__constants['JL'],
            'J0': cls.__constants['J0'],
            'J1': cls.__constants['J1'],
            'J2': cls.__constants['J2'],
            'GRIPPER_TILT': cls.__constants['GRIPPER_TILT'],
            'GRIPPER_ROT': cls.__constants['GRIPPER_ROT'],
            'GRIPPER_CLOSE': cls.__constants['GRIPPER_CLOSE'],
        }

    @property
    def JL(self):
        """Message constant 'JL'."""
        return Metaclass_ArmMsg.__constants['JL']

    @property
    def J0(self):
        """Message constant 'J0'."""
        return Metaclass_ArmMsg.__constants['J0']

    @property
    def J1(self):
        """Message constant 'J1'."""
        return Metaclass_ArmMsg.__constants['J1']

    @property
    def J2(self):
        """Message constant 'J2'."""
        return Metaclass_ArmMsg.__constants['J2']

    @property
    def GRIPPER_TILT(self):
        """Message constant 'GRIPPER_TILT'."""
        return Metaclass_ArmMsg.__constants['GRIPPER_TILT']

    @property
    def GRIPPER_ROT(self):
        """Message constant 'GRIPPER_ROT'."""
        return Metaclass_ArmMsg.__constants['GRIPPER_ROT']

    @property
    def GRIPPER_CLOSE(self):
        """Message constant 'GRIPPER_CLOSE'."""
        return Metaclass_ArmMsg.__constants['GRIPPER_CLOSE']


class ArmMsg(metaclass=Metaclass_ArmMsg):
    """
    Message class 'ArmMsg'.

    Constants:
      JL
      J0
      J1
      J2
      GRIPPER_TILT
      GRIPPER_ROT
      GRIPPER_CLOSE
    """

    __slots__ = [
        '_data',
    ]

    _fields_and_field_types = {
        'data': 'float[7]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 7),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'data' not in kwargs:
            self.data = numpy.zeros(7, dtype=numpy.float32)
        else:
            self.data = numpy.array(kwargs.get('data'), dtype=numpy.float32)
            assert self.data.shape == (7, )

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
        if all(self.data != other.data):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def data(self):
        """Message field 'data'."""
        return self._data

    @data.setter
    def data(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'data' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 7, \
                "The 'data' numpy.ndarray() must have a size of 7"
            self._data = value
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
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'data' field must be a set or sequence with length 7 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._data = numpy.array(value, dtype=numpy.float32)
