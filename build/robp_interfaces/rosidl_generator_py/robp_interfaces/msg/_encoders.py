# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robp_interfaces:msg/Encoders.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Encoders(type):
    """Metaclass of message 'Encoders'."""

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
            module = import_type_support('robp_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robp_interfaces.msg.Encoders')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__encoders
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__encoders
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__encoders
            cls._TYPE_SUPPORT = module.type_support_msg__msg__encoders
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__encoders

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Encoders(metaclass=Metaclass_Encoders):
    """Message class 'Encoders'."""

    __slots__ = [
        '_header',
        '_encoder_left',
        '_encoder_right',
        '_delta_encoder_left',
        '_delta_encoder_right',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'encoder_left': 'int64',
        'encoder_right': 'int64',
        'delta_encoder_left': 'int32',
        'delta_encoder_right': 'int32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.encoder_left = kwargs.get('encoder_left', int())
        self.encoder_right = kwargs.get('encoder_right', int())
        self.delta_encoder_left = kwargs.get('delta_encoder_left', int())
        self.delta_encoder_right = kwargs.get('delta_encoder_right', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.encoder_left != other.encoder_left:
            return False
        if self.encoder_right != other.encoder_right:
            return False
        if self.delta_encoder_left != other.delta_encoder_left:
            return False
        if self.delta_encoder_right != other.delta_encoder_right:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def encoder_left(self):
        """Message field 'encoder_left'."""
        return self._encoder_left

    @encoder_left.setter
    def encoder_left(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'encoder_left' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'encoder_left' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._encoder_left = value

    @builtins.property
    def encoder_right(self):
        """Message field 'encoder_right'."""
        return self._encoder_right

    @encoder_right.setter
    def encoder_right(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'encoder_right' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'encoder_right' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._encoder_right = value

    @builtins.property
    def delta_encoder_left(self):
        """Message field 'delta_encoder_left'."""
        return self._delta_encoder_left

    @delta_encoder_left.setter
    def delta_encoder_left(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'delta_encoder_left' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'delta_encoder_left' field must be an integer in [-2147483648, 2147483647]"
        self._delta_encoder_left = value

    @builtins.property
    def delta_encoder_right(self):
        """Message field 'delta_encoder_right'."""
        return self._delta_encoder_right

    @delta_encoder_right.setter
    def delta_encoder_right(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'delta_encoder_right' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'delta_encoder_right' field must be an integer in [-2147483648, 2147483647]"
        self._delta_encoder_right = value
