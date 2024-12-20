# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:srv/NewGpsGoal.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NewGpsGoal_Request(type):
    """Metaclass of message 'NewGpsGoal_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'GET_ROUTE': 0,
        'NEW_ROUTE': 1,
        'NEW_GOAL_END_APPEND': 2,
        'NEW_GOAL_END_OVERWRITE': 3,
        'NEW_WAYPOINT_BEFORE_END': 4,
        'NEW_WAYPOINT_INDEX_INSERT': 5,
        'NEW_WAYPOINT_INDEX_REPLACE': 6,
        'CLEAR_WAYPOINT_INDEX': 7,
        'CLEAR_ROUTE': 10,
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
                'rover_msgs.srv.NewGpsGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__new_gps_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__new_gps_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__new_gps_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__new_gps_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__new_gps_goal__request

            from rover_msgs.msg import GpsPosition
            if GpsPosition.__class__._TYPE_SUPPORT is None:
                GpsPosition.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'GET_ROUTE': cls.__constants['GET_ROUTE'],
            'NEW_ROUTE': cls.__constants['NEW_ROUTE'],
            'NEW_GOAL_END_APPEND': cls.__constants['NEW_GOAL_END_APPEND'],
            'NEW_GOAL_END_OVERWRITE': cls.__constants['NEW_GOAL_END_OVERWRITE'],
            'NEW_WAYPOINT_BEFORE_END': cls.__constants['NEW_WAYPOINT_BEFORE_END'],
            'NEW_WAYPOINT_INDEX_INSERT': cls.__constants['NEW_WAYPOINT_INDEX_INSERT'],
            'NEW_WAYPOINT_INDEX_REPLACE': cls.__constants['NEW_WAYPOINT_INDEX_REPLACE'],
            'CLEAR_WAYPOINT_INDEX': cls.__constants['CLEAR_WAYPOINT_INDEX'],
            'CLEAR_ROUTE': cls.__constants['CLEAR_ROUTE'],
        }

    @property
    def GET_ROUTE(self):
        """Message constant 'GET_ROUTE'."""
        return Metaclass_NewGpsGoal_Request.__constants['GET_ROUTE']

    @property
    def NEW_ROUTE(self):
        """Message constant 'NEW_ROUTE'."""
        return Metaclass_NewGpsGoal_Request.__constants['NEW_ROUTE']

    @property
    def NEW_GOAL_END_APPEND(self):
        """Message constant 'NEW_GOAL_END_APPEND'."""
        return Metaclass_NewGpsGoal_Request.__constants['NEW_GOAL_END_APPEND']

    @property
    def NEW_GOAL_END_OVERWRITE(self):
        """Message constant 'NEW_GOAL_END_OVERWRITE'."""
        return Metaclass_NewGpsGoal_Request.__constants['NEW_GOAL_END_OVERWRITE']

    @property
    def NEW_WAYPOINT_BEFORE_END(self):
        """Message constant 'NEW_WAYPOINT_BEFORE_END'."""
        return Metaclass_NewGpsGoal_Request.__constants['NEW_WAYPOINT_BEFORE_END']

    @property
    def NEW_WAYPOINT_INDEX_INSERT(self):
        """Message constant 'NEW_WAYPOINT_INDEX_INSERT'."""
        return Metaclass_NewGpsGoal_Request.__constants['NEW_WAYPOINT_INDEX_INSERT']

    @property
    def NEW_WAYPOINT_INDEX_REPLACE(self):
        """Message constant 'NEW_WAYPOINT_INDEX_REPLACE'."""
        return Metaclass_NewGpsGoal_Request.__constants['NEW_WAYPOINT_INDEX_REPLACE']

    @property
    def CLEAR_WAYPOINT_INDEX(self):
        """Message constant 'CLEAR_WAYPOINT_INDEX'."""
        return Metaclass_NewGpsGoal_Request.__constants['CLEAR_WAYPOINT_INDEX']

    @property
    def CLEAR_ROUTE(self):
        """Message constant 'CLEAR_ROUTE'."""
        return Metaclass_NewGpsGoal_Request.__constants['CLEAR_ROUTE']


class NewGpsGoal_Request(metaclass=Metaclass_NewGpsGoal_Request):
    """
    Message class 'NewGpsGoal_Request'.

    Constants:
      GET_ROUTE
      NEW_ROUTE
      NEW_GOAL_END_APPEND
      NEW_GOAL_END_OVERWRITE
      NEW_WAYPOINT_BEFORE_END
      NEW_WAYPOINT_INDEX_INSERT
      NEW_WAYPOINT_INDEX_REPLACE
      CLEAR_WAYPOINT_INDEX
      CLEAR_ROUTE
    """

    __slots__ = [
        '_type',
        '_index',
        '_waypoints',
    ]

    _fields_and_field_types = {
        'type': 'uint8',
        'index': 'uint8',
        'waypoints': 'sequence<rover_msgs/GpsPosition>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rover_msgs', 'msg'], 'GpsPosition')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.type = kwargs.get('type', int())
        self.index = kwargs.get('index', int())
        self.waypoints = kwargs.get('waypoints', [])

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
        if self.type != other.type:
            return False
        if self.index != other.index:
            return False
        if self.waypoints != other.waypoints:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'type' field must be an unsigned integer in [0, 255]"
        self._type = value

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
    def waypoints(self):
        """Message field 'waypoints'."""
        return self._waypoints

    @waypoints.setter
    def waypoints(self, value):
        if __debug__:
            from rover_msgs.msg import GpsPosition
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
                 all(isinstance(v, GpsPosition) for v in value) and
                 True), \
                "The 'waypoints' field must be a set or sequence and each value of type 'GpsPosition'"
        self._waypoints = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NewGpsGoal_Response(type):
    """Metaclass of message 'NewGpsGoal_Response'."""

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
                'rover_msgs.srv.NewGpsGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__new_gps_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__new_gps_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__new_gps_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__new_gps_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__new_gps_goal__response

            from rover_msgs.msg import GpsPosition
            if GpsPosition.__class__._TYPE_SUPPORT is None:
                GpsPosition.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NewGpsGoal_Response(metaclass=Metaclass_NewGpsGoal_Response):
    """Message class 'NewGpsGoal_Response'."""

    __slots__ = [
        '_success',
        '_status',
        '_route',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'status': 'string',
        'route': 'sequence<rover_msgs/GpsPosition>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rover_msgs', 'msg'], 'GpsPosition')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.status = kwargs.get('status', str())
        self.route = kwargs.get('route', [])

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
        if self.status != other.status:
            return False
        if self.route != other.route:
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
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status' field must be of type 'str'"
        self._status = value

    @builtins.property
    def route(self):
        """Message field 'route'."""
        return self._route

    @route.setter
    def route(self, value):
        if __debug__:
            from rover_msgs.msg import GpsPosition
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
                 all(isinstance(v, GpsPosition) for v in value) and
                 True), \
                "The 'route' field must be a set or sequence and each value of type 'GpsPosition'"
        self._route = value


class Metaclass_NewGpsGoal(type):
    """Metaclass of service 'NewGpsGoal'."""

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
                'rover_msgs.srv.NewGpsGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__new_gps_goal

            from rover_msgs.srv import _new_gps_goal
            if _new_gps_goal.Metaclass_NewGpsGoal_Request._TYPE_SUPPORT is None:
                _new_gps_goal.Metaclass_NewGpsGoal_Request.__import_type_support__()
            if _new_gps_goal.Metaclass_NewGpsGoal_Response._TYPE_SUPPORT is None:
                _new_gps_goal.Metaclass_NewGpsGoal_Response.__import_type_support__()


class NewGpsGoal(metaclass=Metaclass_NewGpsGoal):
    from rover_msgs.srv._new_gps_goal import NewGpsGoal_Request as Request
    from rover_msgs.srv._new_gps_goal import NewGpsGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
