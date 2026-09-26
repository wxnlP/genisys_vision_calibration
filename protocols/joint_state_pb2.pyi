from . import header_pb2 as _header_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class JointState(_message.Message):
    __slots__ = ("name", "position", "velocity", "effort")
    NAME_FIELD_NUMBER: _ClassVar[int]
    POSITION_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_FIELD_NUMBER: _ClassVar[int]
    EFFORT_FIELD_NUMBER: _ClassVar[int]
    name: str
    position: float
    velocity: float
    effort: float
    def __init__(self, name: _Optional[str] = ..., position: _Optional[float] = ..., velocity: _Optional[float] = ..., effort: _Optional[float] = ...) -> None: ...

class JointStateArray(_message.Message):
    __slots__ = ("header", "joints")
    HEADER_FIELD_NUMBER: _ClassVar[int]
    JOINTS_FIELD_NUMBER: _ClassVar[int]
    header: _header_pb2.Header
    joints: _containers.RepeatedCompositeFieldContainer[JointState]
    def __init__(self, header: _Optional[_Union[_header_pb2.Header, _Mapping]] = ..., joints: _Optional[_Iterable[_Union[JointState, _Mapping]]] = ...) -> None: ...
