# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:srv/DriveTrainArbitration.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DriveTrainArbitration_Request(type):
    """Metaclass of message 'DriveTrainArbitration_Request'."""

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
                'rover_msgs.srv.DriveTrainArbitration_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__drive_train_arbitration__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__drive_train_arbitration__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__drive_train_arbitration__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__drive_train_arbitration__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__drive_train_arbitration__request

            from rover_msgs.msg import DrivetrainArbitration
            if DrivetrainArbitration.__class__._TYPE_SUPPORT is None:
                DrivetrainArbitration.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DriveTrainArbitration_Request(metaclass=Metaclass_DriveTrainArbitration_Request):
    """Message class 'DriveTrainArbitration_Request'."""

    __slots__ = [
        '_target_arbitration',
    ]

    _fields_and_field_types = {
        'target_arbitration': 'rover_msgs/DrivetrainArbitration',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['rover_msgs', 'msg'], 'DrivetrainArbitration'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from rover_msgs.msg import DrivetrainArbitration
        self.target_arbitration = kwargs.get('target_arbitration', DrivetrainArbitration())

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
        if self.target_arbitration != other.target_arbitration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def target_arbitration(self):
        """Message field 'target_arbitration'."""
        return self._target_arbitration

    @target_arbitration.setter
    def target_arbitration(self, value):
        if __debug__:
            from rover_msgs.msg import DrivetrainArbitration
            assert \
                isinstance(value, DrivetrainArbitration), \
                "The 'target_arbitration' field must be a sub message of type 'DrivetrainArbitration'"
        self._target_arbitration = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_DriveTrainArbitration_Response(type):
    """Metaclass of message 'DriveTrainArbitration_Response'."""

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
                'rover_msgs.srv.DriveTrainArbitration_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__drive_train_arbitration__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__drive_train_arbitration__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__drive_train_arbitration__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__drive_train_arbitration__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__drive_train_arbitration__response

            from rover_msgs.msg import DrivetrainArbitration
            if DrivetrainArbitration.__class__._TYPE_SUPPORT is None:
                DrivetrainArbitration.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DriveTrainArbitration_Response(metaclass=Metaclass_DriveTrainArbitration_Response):
    """Message class 'DriveTrainArbitration_Response'."""

    __slots__ = [
        '_current_arbitration',
    ]

    _fields_and_field_types = {
        'current_arbitration': 'rover_msgs/DrivetrainArbitration',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['rover_msgs', 'msg'], 'DrivetrainArbitration'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from rover_msgs.msg import DrivetrainArbitration
        self.current_arbitration = kwargs.get('current_arbitration', DrivetrainArbitration())

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
        if self.current_arbitration != other.current_arbitration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def current_arbitration(self):
        """Message field 'current_arbitration'."""
        return self._current_arbitration

    @current_arbitration.setter
    def current_arbitration(self, value):
        if __debug__:
            from rover_msgs.msg import DrivetrainArbitration
            assert \
                isinstance(value, DrivetrainArbitration), \
                "The 'current_arbitration' field must be a sub message of type 'DrivetrainArbitration'"
        self._current_arbitration = value


class Metaclass_DriveTrainArbitration(type):
    """Metaclass of service 'DriveTrainArbitration'."""

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
                'rover_msgs.srv.DriveTrainArbitration')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__drive_train_arbitration

            from rover_msgs.srv import _drive_train_arbitration
            if _drive_train_arbitration.Metaclass_DriveTrainArbitration_Request._TYPE_SUPPORT is None:
                _drive_train_arbitration.Metaclass_DriveTrainArbitration_Request.__import_type_support__()
            if _drive_train_arbitration.Metaclass_DriveTrainArbitration_Response._TYPE_SUPPORT is None:
                _drive_train_arbitration.Metaclass_DriveTrainArbitration_Response.__import_type_support__()


class DriveTrainArbitration(metaclass=Metaclass_DriveTrainArbitration):
    from rover_msgs.srv._drive_train_arbitration import DriveTrainArbitration_Request as Request
    from rover_msgs.srv._drive_train_arbitration import DriveTrainArbitration_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
