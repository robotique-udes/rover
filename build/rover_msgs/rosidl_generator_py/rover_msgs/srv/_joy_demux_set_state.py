# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:srv/JoyDemuxSetState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_JoyDemuxSetState_Request(type):
    """Metaclass of message 'JoyDemuxSetState_Request'."""

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
                'rover_msgs.srv.JoyDemuxSetState_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__joy_demux_set_state__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__joy_demux_set_state__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__joy_demux_set_state__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__joy_demux_set_state__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__joy_demux_set_state__request

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
        return Metaclass_JoyDemuxSetState_Request.__constants['CONTROLLER_MAIN']

    @property
    def CONTROLLER_SECONDARY(self):
        """Message constant 'CONTROLLER_SECONDARY'."""
        return Metaclass_JoyDemuxSetState_Request.__constants['CONTROLLER_SECONDARY']

    @property
    def DEST_DRIVE_TRAIN(self):
        """Message constant 'DEST_DRIVE_TRAIN'."""
        return Metaclass_JoyDemuxSetState_Request.__constants['DEST_DRIVE_TRAIN']

    @property
    def DEST_ARM(self):
        """Message constant 'DEST_ARM'."""
        return Metaclass_JoyDemuxSetState_Request.__constants['DEST_ARM']

    @property
    def DEST_ANTENNA(self):
        """Message constant 'DEST_ANTENNA'."""
        return Metaclass_JoyDemuxSetState_Request.__constants['DEST_ANTENNA']

    @property
    def DEST_NONE(self):
        """Message constant 'DEST_NONE'."""
        return Metaclass_JoyDemuxSetState_Request.__constants['DEST_NONE']


class JoyDemuxSetState_Request(metaclass=Metaclass_JoyDemuxSetState_Request):
    """
    Message class 'JoyDemuxSetState_Request'.

    Constants:
      CONTROLLER_MAIN
      CONTROLLER_SECONDARY
      DEST_DRIVE_TRAIN
      DEST_ARM
      DEST_ANTENNA
      DEST_NONE
    """

    __slots__ = [
        '_controller_type',
        '_destination',
        '_force',
    ]

    _fields_and_field_types = {
        'controller_type': 'uint8',
        'destination': 'uint8',
        'force': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.controller_type = kwargs.get('controller_type', int())
        self.destination = kwargs.get('destination', int())
        self.force = kwargs.get('force', bool())

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
        if self.controller_type != other.controller_type:
            return False
        if self.destination != other.destination:
            return False
        if self.force != other.force:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def controller_type(self):
        """Message field 'controller_type'."""
        return self._controller_type

    @controller_type.setter
    def controller_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'controller_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'controller_type' field must be an unsigned integer in [0, 255]"
        self._controller_type = value

    @builtins.property
    def destination(self):
        """Message field 'destination'."""
        return self._destination

    @destination.setter
    def destination(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'destination' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'destination' field must be an unsigned integer in [0, 255]"
        self._destination = value

    @builtins.property
    def force(self):
        """Message field 'force'."""
        return self._force

    @force.setter
    def force(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'force' field must be of type 'bool'"
        self._force = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_JoyDemuxSetState_Response(type):
    """Metaclass of message 'JoyDemuxSetState_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
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
                'rover_msgs.srv.JoyDemuxSetState_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__joy_demux_set_state__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__joy_demux_set_state__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__joy_demux_set_state__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__joy_demux_set_state__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__joy_demux_set_state__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class JoyDemuxSetState_Response(metaclass=Metaclass_JoyDemuxSetState_Response):
    """Message class 'JoyDemuxSetState_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_JoyDemuxSetState(type):
    """Metaclass of service 'JoyDemuxSetState'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rover_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rover_msgs.srv.JoyDemuxSetState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__joy_demux_set_state

            from rover_msgs.srv import _joy_demux_set_state
            if _joy_demux_set_state.Metaclass_JoyDemuxSetState_Request._TYPE_SUPPORT is None:
                _joy_demux_set_state.Metaclass_JoyDemuxSetState_Request.__import_type_support__()
            if _joy_demux_set_state.Metaclass_JoyDemuxSetState_Response._TYPE_SUPPORT is None:
                _joy_demux_set_state.Metaclass_JoyDemuxSetState_Response.__import_type_support__()


class JoyDemuxSetState(metaclass=Metaclass_JoyDemuxSetState):
    from rover_msgs.srv._joy_demux_set_state import JoyDemuxSetState_Request as Request
    from rover_msgs.srv._joy_demux_set_state import JoyDemuxSetState_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
