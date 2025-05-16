# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:srv/LightControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LightControl_Request(type):
    """Metaclass of message 'LightControl_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'LIGHT': 0,
        'LIGHT_INFRARED': 1,
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
                'rover_msgs.srv.LightControl_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__light_control__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__light_control__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__light_control__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__light_control__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__light_control__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'LIGHT': cls.__constants['LIGHT'],
            'LIGHT_INFRARED': cls.__constants['LIGHT_INFRARED'],
        }

    @property
    def LIGHT(self):
        """Message constant 'LIGHT'."""
        return Metaclass_LightControl_Request.__constants['LIGHT']

    @property
    def LIGHT_INFRARED(self):
        """Message constant 'LIGHT_INFRARED'."""
        return Metaclass_LightControl_Request.__constants['LIGHT_INFRARED']


class LightControl_Request(metaclass=Metaclass_LightControl_Request):
    """
    Message class 'LightControl_Request'.

    Constants:
      LIGHT
      LIGHT_INFRARED
    """

    __slots__ = [
        '_index',
        '_enable',
    ]

    _fields_and_field_types = {
        'index': 'uint8',
        'enable': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.index = kwargs.get('index', int())
        self.enable = kwargs.get('enable', bool())

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
        if self.index != other.index:
            return False
        if self.enable != other.enable:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def index(self):
        """Message field 'index'."""
        return self._index

    @index.setter
    def index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'index' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'index' field must be an unsigned integer in [0, 255]"
        self._index = value

    @builtins.property
    def enable(self):
        """Message field 'enable'."""
        return self._enable

    @enable.setter
    def enable(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'enable' field must be of type 'bool'"
        self._enable = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_LightControl_Response(type):
    """Metaclass of message 'LightControl_Response'."""

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
                'rover_msgs.srv.LightControl_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__light_control__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__light_control__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__light_control__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__light_control__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__light_control__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LightControl_Response(metaclass=Metaclass_LightControl_Response):
    """Message class 'LightControl_Response'."""

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


class Metaclass_LightControl(type):
    """Metaclass of service 'LightControl'."""

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
                'rover_msgs.srv.LightControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__light_control

            from rover_msgs.srv import _light_control
            if _light_control.Metaclass_LightControl_Request._TYPE_SUPPORT is None:
                _light_control.Metaclass_LightControl_Request.__import_type_support__()
            if _light_control.Metaclass_LightControl_Response._TYPE_SUPPORT is None:
                _light_control.Metaclass_LightControl_Response.__import_type_support__()


class LightControl(metaclass=Metaclass_LightControl):
    from rover_msgs.srv._light_control import LightControl_Request as Request
    from rover_msgs.srv._light_control import LightControl_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
