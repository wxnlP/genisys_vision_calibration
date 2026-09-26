from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class Header(_message.Message):
    __slots__ = ("time_stamp", "frame_id", "sequence_num")
    TIME_STAMP_FIELD_NUMBER: _ClassVar[int]
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    SEQUENCE_NUM_FIELD_NUMBER: _ClassVar[int]
    time_stamp: int
    frame_id: str
    sequence_num: int
    def __init__(self, time_stamp: _Optional[int] = ..., frame_id: _Optional[str] = ..., sequence_num: _Optional[int] = ...) -> None: ...
