# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:srv/PanoControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PanoControl_Request(type):
    """Metaclass of message 'PanoControl_Request'."""

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
                'rover_msgs.srv.PanoControl_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__pano_control__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__pano_control__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__pano_control__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__pano_control__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__pano_control__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PanoControl_Request(metaclass=Metaclass_PanoControl_Request):
    """Message class 'PanoControl_Request'."""

    __slots__ = [
        '_start',
        '_stop',
        '_photo',
        '_ip_address',
    ]

    _fields_and_field_types = {
        'start': 'boolean',
        'stop': 'boolean',
        'photo': 'boolean',
        'ip_address': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.start = kwargs.get('start', bool())
        self.stop = kwargs.get('stop', bool())
        self.photo = kwargs.get('photo', bool())
        self.ip_address = kwargs.get('ip_address', str())

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
        if self.start != other.start:
            return False
        if self.stop != other.stop:
            return False
        if self.photo != other.photo:
            return False
        if self.ip_address != other.ip_address:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def start(self):
        """Message field 'start'."""
        return self._start

    @start.setter
    def start(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'start' field must be of type 'bool'"
        self._start = value

    @builtins.property
    def stop(self):
        """Message field 'stop'."""
        return self._stop

    @stop.setter
    def stop(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'stop' field must be of type 'bool'"
        self._stop = value

    @builtins.property
    def photo(self):
        """Message field 'photo'."""
        return self._photo

    @photo.setter
    def photo(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'photo' field must be of type 'bool'"
        self._photo = value

    @builtins.property
    def ip_address(self):
        """Message field 'ip_address'."""
        return self._ip_address

    @ip_address.setter
    def ip_address(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'ip_address' field must be of type 'str'"
        self._ip_address = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PanoControl_Response(type):
    """Metaclass of message 'PanoControl_Response'."""

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
                'rover_msgs.srv.PanoControl_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__pano_control__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__pano_control__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__pano_control__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__pano_control__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__pano_control__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PanoControl_Response(metaclass=Metaclass_PanoControl_Response):
    """Message class 'PanoControl_Response'."""

    __slots__ = [
        '_success',
        '_status_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'status_message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.status_message = kwargs.get('status_message', str())

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
        if self.status_message != other.status_message:
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

    @builtins.property
    def status_message(self):
        """Message field 'status_message'."""
        return self._status_message

    @status_message.setter
    def status_message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status_message' field must be of type 'str'"
        self._status_message = value


class Metaclass_PanoControl(type):
    """Metaclass of service 'PanoControl'."""

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
                'rover_msgs.srv.PanoControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__pano_control

            from rover_msgs.srv import _pano_control
            if _pano_control.Metaclass_PanoControl_Request._TYPE_SUPPORT is None:
                _pano_control.Metaclass_PanoControl_Request.__import_type_support__()
            if _pano_control.Metaclass_PanoControl_Response._TYPE_SUPPORT is None:
                _pano_control.Metaclass_PanoControl_Response.__import_type_support__()


class PanoControl(metaclass=Metaclass_PanoControl):
    from rover_msgs.srv._pano_control import PanoControl_Request as Request
    from rover_msgs.srv._pano_control import PanoControl_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
