# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:srv/RtspStream.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RtspStream_Request(type):
    """Metaclass of message 'RtspStream_Request'."""

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
                'rover_msgs.srv.RtspStream_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__rtsp_stream__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__rtsp_stream__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__rtsp_stream__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__rtsp_stream__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__rtsp_stream__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RtspStream_Request(metaclass=Metaclass_RtspStream_Request):
    """Message class 'RtspStream_Request'."""

    __slots__ = [
        '_stream_id',
        '_demand',
    ]

    _fields_and_field_types = {
        'stream_id': 'string',
        'demand': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.stream_id = kwargs.get('stream_id', str())
        self.demand = kwargs.get('demand', bool())

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
        if self.stream_id != other.stream_id:
            return False
        if self.demand != other.demand:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def stream_id(self):
        """Message field 'stream_id'."""
        return self._stream_id

    @stream_id.setter
    def stream_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'stream_id' field must be of type 'str'"
        self._stream_id = value

    @builtins.property
    def demand(self):
        """Message field 'demand'."""
        return self._demand

    @demand.setter
    def demand(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'demand' field must be of type 'bool'"
        self._demand = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_RtspStream_Response(type):
    """Metaclass of message 'RtspStream_Response'."""

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
                'rover_msgs.srv.RtspStream_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__rtsp_stream__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__rtsp_stream__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__rtsp_stream__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__rtsp_stream__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__rtsp_stream__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RtspStream_Response(metaclass=Metaclass_RtspStream_Response):
    """Message class 'RtspStream_Response'."""

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


class Metaclass_RtspStream(type):
    """Metaclass of service 'RtspStream'."""

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
                'rover_msgs.srv.RtspStream')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__rtsp_stream

            from rover_msgs.srv import _rtsp_stream
            if _rtsp_stream.Metaclass_RtspStream_Request._TYPE_SUPPORT is None:
                _rtsp_stream.Metaclass_RtspStream_Request.__import_type_support__()
            if _rtsp_stream.Metaclass_RtspStream_Response._TYPE_SUPPORT is None:
                _rtsp_stream.Metaclass_RtspStream_Response.__import_type_support__()


class RtspStream(metaclass=Metaclass_RtspStream):
    from rover_msgs.srv._rtsp_stream import RtspStream_Request as Request
    from rover_msgs.srv._rtsp_stream import RtspStream_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
