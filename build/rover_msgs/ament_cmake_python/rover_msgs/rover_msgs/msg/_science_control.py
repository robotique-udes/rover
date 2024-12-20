# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:msg/ScienceControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ScienceControl(type):
    """Metaclass of message 'ScienceControl'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'DOWN': 0,
        'UP': 1,
        'E1': 0,
        'E2': 1,
        'E3': 2,
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
                'rover_msgs.msg.ScienceControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__science_control
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__science_control
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__science_control
            cls._TYPE_SUPPORT = module.type_support_msg__msg__science_control
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__science_control

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'DOWN': cls.__constants['DOWN'],
            'UP': cls.__constants['UP'],
            'E1': cls.__constants['E1'],
            'E2': cls.__constants['E2'],
            'E3': cls.__constants['E3'],
        }

    @property
    def DOWN(self):
        """Message constant 'DOWN'."""
        return Metaclass_ScienceControl.__constants['DOWN']

    @property
    def UP(self):
        """Message constant 'UP'."""
        return Metaclass_ScienceControl.__constants['UP']

    @property
    def E1(self):
        """Message constant 'E1'."""
        return Metaclass_ScienceControl.__constants['E1']

    @property
    def E2(self):
        """Message constant 'E2'."""
        return Metaclass_ScienceControl.__constants['E2']

    @property
    def E3(self):
        """Message constant 'E3'."""
        return Metaclass_ScienceControl.__constants['E3']


class ScienceControl(metaclass=Metaclass_ScienceControl):
    """
    Message class 'ScienceControl'.

    Constants:
      DOWN
      UP
      E1
      E2
      E3
    """

    __slots__ = [
        '_cmd',
        '_current_sample',
        '_dig',
    ]

    _fields_and_field_types = {
        'cmd': 'int8',
        'current_sample': 'int8',
        'dig': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cmd = kwargs.get('cmd', int())
        self.current_sample = kwargs.get('current_sample', int())
        self.dig = kwargs.get('dig', bool())

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
        if self.cmd != other.cmd:
            return False
        if self.current_sample != other.current_sample:
            return False
        if self.dig != other.dig:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cmd(self):
        """Message field 'cmd'."""
        return self._cmd

    @cmd.setter
    def cmd(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cmd' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'cmd' field must be an integer in [-128, 127]"
        self._cmd = value

    @builtins.property
    def current_sample(self):
        """Message field 'current_sample'."""
        return self._current_sample

    @current_sample.setter
    def current_sample(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_sample' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'current_sample' field must be an integer in [-128, 127]"
        self._current_sample = value

    @builtins.property
    def dig(self):
        """Message field 'dig'."""
        return self._dig

    @dig.setter
    def dig(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'dig' field must be of type 'bool'"
        self._dig = value
