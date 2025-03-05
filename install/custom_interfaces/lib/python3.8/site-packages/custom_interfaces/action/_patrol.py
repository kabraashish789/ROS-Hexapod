# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_interfaces:action/Patrol.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Patrol_Goal(type):
    """Metaclass of message 'Patrol_Goal'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.Patrol_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__patrol__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__patrol__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__patrol__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__patrol__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__patrol__goal

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Patrol_Goal(metaclass=Metaclass_Patrol_Goal):
    """Message class 'Patrol_Goal'."""

    __slots__ = [
        '_revolut1',
        '_revolut2',
        '_revolut3',
        '_revolut4',
        '_revolut5',
        '_revolut6',
        '_revolut7',
        '_revolut8',
        '_revolut9',
        '_leg_number',
    ]

    _fields_and_field_types = {
        'revolut1': 'double',
        'revolut2': 'double',
        'revolut3': 'double',
        'revolut4': 'double',
        'revolut5': 'double',
        'revolut6': 'double',
        'revolut7': 'double',
        'revolut8': 'double',
        'revolut9': 'double',
        'leg_number': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.revolut1 = kwargs.get('revolut1', float())
        self.revolut2 = kwargs.get('revolut2', float())
        self.revolut3 = kwargs.get('revolut3', float())
        self.revolut4 = kwargs.get('revolut4', float())
        self.revolut5 = kwargs.get('revolut5', float())
        self.revolut6 = kwargs.get('revolut6', float())
        self.revolut7 = kwargs.get('revolut7', float())
        self.revolut8 = kwargs.get('revolut8', float())
        self.revolut9 = kwargs.get('revolut9', float())
        self.leg_number = kwargs.get('leg_number', float())

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
        if self.revolut1 != other.revolut1:
            return False
        if self.revolut2 != other.revolut2:
            return False
        if self.revolut3 != other.revolut3:
            return False
        if self.revolut4 != other.revolut4:
            return False
        if self.revolut5 != other.revolut5:
            return False
        if self.revolut6 != other.revolut6:
            return False
        if self.revolut7 != other.revolut7:
            return False
        if self.revolut8 != other.revolut8:
            return False
        if self.revolut9 != other.revolut9:
            return False
        if self.leg_number != other.leg_number:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def revolut1(self):
        """Message field 'revolut1'."""
        return self._revolut1

    @revolut1.setter
    def revolut1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'revolut1' field must be of type 'float'"
        self._revolut1 = value

    @property
    def revolut2(self):
        """Message field 'revolut2'."""
        return self._revolut2

    @revolut2.setter
    def revolut2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'revolut2' field must be of type 'float'"
        self._revolut2 = value

    @property
    def revolut3(self):
        """Message field 'revolut3'."""
        return self._revolut3

    @revolut3.setter
    def revolut3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'revolut3' field must be of type 'float'"
        self._revolut3 = value

    @property
    def revolut4(self):
        """Message field 'revolut4'."""
        return self._revolut4

    @revolut4.setter
    def revolut4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'revolut4' field must be of type 'float'"
        self._revolut4 = value

    @property
    def revolut5(self):
        """Message field 'revolut5'."""
        return self._revolut5

    @revolut5.setter
    def revolut5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'revolut5' field must be of type 'float'"
        self._revolut5 = value

    @property
    def revolut6(self):
        """Message field 'revolut6'."""
        return self._revolut6

    @revolut6.setter
    def revolut6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'revolut6' field must be of type 'float'"
        self._revolut6 = value

    @property
    def revolut7(self):
        """Message field 'revolut7'."""
        return self._revolut7

    @revolut7.setter
    def revolut7(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'revolut7' field must be of type 'float'"
        self._revolut7 = value

    @property
    def revolut8(self):
        """Message field 'revolut8'."""
        return self._revolut8

    @revolut8.setter
    def revolut8(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'revolut8' field must be of type 'float'"
        self._revolut8 = value

    @property
    def revolut9(self):
        """Message field 'revolut9'."""
        return self._revolut9

    @revolut9.setter
    def revolut9(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'revolut9' field must be of type 'float'"
        self._revolut9 = value

    @property
    def leg_number(self):
        """Message field 'leg_number'."""
        return self._leg_number

    @leg_number.setter
    def leg_number(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'leg_number' field must be of type 'float'"
        self._leg_number = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_Patrol_Result(type):
    """Metaclass of message 'Patrol_Result'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.Patrol_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__patrol__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__patrol__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__patrol__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__patrol__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__patrol__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Patrol_Result(metaclass=Metaclass_Patrol_Result):
    """Message class 'Patrol_Result'."""

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

    @property
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


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_Patrol_Feedback(type):
    """Metaclass of message 'Patrol_Feedback'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.Patrol_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__patrol__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__patrol__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__patrol__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__patrol__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__patrol__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Patrol_Feedback(metaclass=Metaclass_Patrol_Feedback):
    """Message class 'Patrol_Feedback'."""

    __slots__ = [
        '_time_left',
    ]

    _fields_and_field_types = {
        'time_left': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.time_left = kwargs.get('time_left', float())

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
        if self.time_left != other.time_left:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def time_left(self):
        """Message field 'time_left'."""
        return self._time_left

    @time_left.setter
    def time_left(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'time_left' field must be of type 'float'"
        self._time_left = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_Patrol_SendGoal_Request(type):
    """Metaclass of message 'Patrol_SendGoal_Request'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.Patrol_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__patrol__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__patrol__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__patrol__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__patrol__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__patrol__send_goal__request

            from custom_interfaces.action import Patrol
            if Patrol.Goal.__class__._TYPE_SUPPORT is None:
                Patrol.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Patrol_SendGoal_Request(metaclass=Metaclass_Patrol_SendGoal_Request):
    """Message class 'Patrol_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'custom_interfaces/Patrol_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['custom_interfaces', 'action'], 'Patrol_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from custom_interfaces.action._patrol import Patrol_Goal
        self.goal = kwargs.get('goal', Patrol_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from custom_interfaces.action._patrol import Patrol_Goal
            assert \
                isinstance(value, Patrol_Goal), \
                "The 'goal' field must be a sub message of type 'Patrol_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_Patrol_SendGoal_Response(type):
    """Metaclass of message 'Patrol_SendGoal_Response'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.Patrol_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__patrol__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__patrol__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__patrol__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__patrol__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__patrol__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Patrol_SendGoal_Response(metaclass=Metaclass_Patrol_SendGoal_Response):
    """Message class 'Patrol_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_Patrol_SendGoal(type):
    """Metaclass of service 'Patrol_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.Patrol_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__patrol__send_goal

            from custom_interfaces.action import _patrol
            if _patrol.Metaclass_Patrol_SendGoal_Request._TYPE_SUPPORT is None:
                _patrol.Metaclass_Patrol_SendGoal_Request.__import_type_support__()
            if _patrol.Metaclass_Patrol_SendGoal_Response._TYPE_SUPPORT is None:
                _patrol.Metaclass_Patrol_SendGoal_Response.__import_type_support__()


class Patrol_SendGoal(metaclass=Metaclass_Patrol_SendGoal):
    from custom_interfaces.action._patrol import Patrol_SendGoal_Request as Request
    from custom_interfaces.action._patrol import Patrol_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_Patrol_GetResult_Request(type):
    """Metaclass of message 'Patrol_GetResult_Request'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.Patrol_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__patrol__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__patrol__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__patrol__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__patrol__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__patrol__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Patrol_GetResult_Request(metaclass=Metaclass_Patrol_GetResult_Request):
    """Message class 'Patrol_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_Patrol_GetResult_Response(type):
    """Metaclass of message 'Patrol_GetResult_Response'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.Patrol_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__patrol__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__patrol__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__patrol__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__patrol__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__patrol__get_result__response

            from custom_interfaces.action import Patrol
            if Patrol.Result.__class__._TYPE_SUPPORT is None:
                Patrol.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Patrol_GetResult_Response(metaclass=Metaclass_Patrol_GetResult_Response):
    """Message class 'Patrol_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'custom_interfaces/Patrol_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['custom_interfaces', 'action'], 'Patrol_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from custom_interfaces.action._patrol import Patrol_Result
        self.result = kwargs.get('result', Patrol_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from custom_interfaces.action._patrol import Patrol_Result
            assert \
                isinstance(value, Patrol_Result), \
                "The 'result' field must be a sub message of type 'Patrol_Result'"
        self._result = value


class Metaclass_Patrol_GetResult(type):
    """Metaclass of service 'Patrol_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.Patrol_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__patrol__get_result

            from custom_interfaces.action import _patrol
            if _patrol.Metaclass_Patrol_GetResult_Request._TYPE_SUPPORT is None:
                _patrol.Metaclass_Patrol_GetResult_Request.__import_type_support__()
            if _patrol.Metaclass_Patrol_GetResult_Response._TYPE_SUPPORT is None:
                _patrol.Metaclass_Patrol_GetResult_Response.__import_type_support__()


class Patrol_GetResult(metaclass=Metaclass_Patrol_GetResult):
    from custom_interfaces.action._patrol import Patrol_GetResult_Request as Request
    from custom_interfaces.action._patrol import Patrol_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_Patrol_FeedbackMessage(type):
    """Metaclass of message 'Patrol_FeedbackMessage'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.Patrol_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__patrol__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__patrol__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__patrol__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__patrol__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__patrol__feedback_message

            from custom_interfaces.action import Patrol
            if Patrol.Feedback.__class__._TYPE_SUPPORT is None:
                Patrol.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Patrol_FeedbackMessage(metaclass=Metaclass_Patrol_FeedbackMessage):
    """Message class 'Patrol_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'custom_interfaces/Patrol_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['custom_interfaces', 'action'], 'Patrol_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from custom_interfaces.action._patrol import Patrol_Feedback
        self.feedback = kwargs.get('feedback', Patrol_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from custom_interfaces.action._patrol import Patrol_Feedback
            assert \
                isinstance(value, Patrol_Feedback), \
                "The 'feedback' field must be a sub message of type 'Patrol_Feedback'"
        self._feedback = value


class Metaclass_Patrol(type):
    """Metaclass of action 'Patrol'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.Patrol')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__patrol

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from custom_interfaces.action import _patrol
            if _patrol.Metaclass_Patrol_SendGoal._TYPE_SUPPORT is None:
                _patrol.Metaclass_Patrol_SendGoal.__import_type_support__()
            if _patrol.Metaclass_Patrol_GetResult._TYPE_SUPPORT is None:
                _patrol.Metaclass_Patrol_GetResult.__import_type_support__()
            if _patrol.Metaclass_Patrol_FeedbackMessage._TYPE_SUPPORT is None:
                _patrol.Metaclass_Patrol_FeedbackMessage.__import_type_support__()


class Patrol(metaclass=Metaclass_Patrol):

    # The goal message defined in the action definition.
    from custom_interfaces.action._patrol import Patrol_Goal as Goal
    # The result message defined in the action definition.
    from custom_interfaces.action._patrol import Patrol_Result as Result
    # The feedback message defined in the action definition.
    from custom_interfaces.action._patrol import Patrol_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from custom_interfaces.action._patrol import Patrol_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from custom_interfaces.action._patrol import Patrol_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from custom_interfaces.action._patrol import Patrol_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
