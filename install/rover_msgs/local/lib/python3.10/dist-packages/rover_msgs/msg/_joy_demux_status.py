# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:msg/JoyDemuxStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_JoyDemuxStatus(type):
    """Metaclass of message 'JoyDemuxStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CONTROLLER_MAIN': 0,
        'CONTROLLER_SECONDARY': 1,
        'DEST_DRIVE_TRAIN': 0,
        'DEST_ARM': 1,
        'DEST_ANTENNA': 2,
        'DEST_NONE': 3,
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
                'rover_msgs.msg.JoyDemuxStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__joy_demux_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__joy_demux_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__joy_demux_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__joy_demux_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__joy_demux_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CONTROLLER_MAIN': cls.__constants['CONTROLLER_MAIN'],
            'CONTROLLER_SECONDARY': cls.__constants['CONTROLLER_SECONDARY'],
            'DEST_DRIVE_TRAIN': cls.__constants['DEST_DRIVE_TRAIN'],
            'DEST_ARM': cls.__constants['DEST_ARM'],
            'DEST_ANTENNA': cls.__constants['DEST_ANTENNA'],
            'DEST_NONE': cls.__constants['DEST_NONE'],
        }

    @property
    def CONTROLLER_MAIN(self):
        """Message constant 'CONTROLLER_MAIN'."""
        return Metaclass_JoyDemuxStatus.__constants['CONTROLLER_MAIN']

    @property
    def CONTROLLER_SECONDARY(self):
        """Message constant 'CONTROLLER_SECONDARY'."""
        return Metaclass_JoyDemuxStatus.__constants['CONTROLLER_SECONDARY']

    @property
    def DEST_DRIVE_TRAIN(self):
        """Message constant 'DEST_DRIVE_TRAIN'."""
        return Metaclass_JoyDemuxStatus.__constants['DEST_DRIVE_TRAIN']

    @property
    def DEST_ARM(self):
        """Message constant 'DEST_ARM'."""
        return Metaclass_JoyDemuxStatus.__constants['DEST_ARM']

    @property
    def DEST_ANTENNA(self):
        """Message constant 'DEST_ANTENNA'."""
        return Metaclass_JoyDemuxStatus.__constants['DEST_ANTENNA']

    @property
    def DEST_NONE(self):
        """Message constant 'DEST_NONE'."""
        return Metaclass_JoyDemuxStatus.__constants['DEST_NONE']


class JoyDemuxStatus(metaclass=Metaclass_JoyDemuxStatus):
    """
    Message class 'JoyDemuxStatus'.

    Constants:
      CONTROLLER_MAIN
      CONTROLLER_SECONDARY
      DEST_DRIVE_TRAIN
      DEST_ARM
      DEST_ANTENNA
      DEST_NONE
    """

    __slots__ = [
        '_controller_main_topic',
        '_controller_secondary_topic',
    ]

    _fields_and_field_types = {
        'controller_main_topic': 'uint8',
        'controller_secondary_topic': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.controller_main_topic = kwargs.get('controller_main_topic', int())
        self.controller_secondary_topic = kwargs.get('controller_secondary_topic', int())

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
        if self.controller_main_topic != other.controller_main_topic:
            return False
        if self.controller_secondary_topic != other.controller_secondary_topic:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def controller_main_topic(self):
        """Message field 'controller_main_topic'."""
        return self._controller_main_topic

    @controller_main_topic.setter
    def controller_main_topic(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'controller_main_topic' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'controller_main_topic' field must be an unsigned integer in [0, 255]"
        self._controller_main_topic = value

    @builtins.property
    def controller_secondary_topic(self):
        """Message field 'controller_secondary_topic'."""
        return self._controller_secondary_topic

    @controller_secondary_topic.setter
    def controller_secondary_topic(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'controller_secondary_topic' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'controller_secondary_topic' field must be an unsigned integer in [0, 255]"
        self._controller_secondary_topic = value
