from . import header_pb2 as _header_pb2
from . import quaternion_pb2 as _quaternion_pb2
from . import vector3_pb2 as _vector3_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Pose(_message.Message):
    __slots__ = ("orientation", "position")
    ORIENTATION_FIELD_NUMBER: _ClassVar[int]
    POSITION_FIELD_NUMBER: _ClassVar[int]
    orientation: _quaternion_pb2.Quaternion
    position: _vector3_pb2.Vector3
    def __init__(self, orientation: _Optional[_Union[_quaternion_pb2.Quaternion, _Mapping]] = ..., position: _Optional[_Union[_vector3_pb2.Vector3, _Mapping]] = ...) -> None: ...

class PoseStamped(_message.Message):
    __slots__ = ("header", "pose")
    HEADER_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    header: _header_pb2.Header
    pose: Pose
    def __init__(self, header: _Optional[_Union[_header_pb2.Header, _Mapping]] = ..., pose: _Optional[_Union[Pose, _Mapping]] = ...) -> None: ...
