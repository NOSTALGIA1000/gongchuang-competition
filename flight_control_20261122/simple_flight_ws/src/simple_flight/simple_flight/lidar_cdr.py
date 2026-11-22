"""Bounded access to the pinned driver's CustomMsg in ROS 2 XCDR1 format.

Avoid creating 20,000 Python point objects for every cloud. Layout follows
CustomMsg.msg and CustomPoint.msg; alignment is relative to the CDR body.
"""

import struct


class CloudView:
    def __init__(self, raw):
        if len(raw) < 20 or raw[:2] not in (b'\0\0', b'\0\1'):
            raise ValueError('unsupported or truncated CDR encoding')
        self.body = memoryview(raw)[4:]
        self.order = '<' if raw[1] == 1 else '>'
        sec, nano, size = struct.unpack_from(self.order + 'iII', self.body)
        if size < 1 or 12 + size > len(self.body) or self.body[11 + size] != 0:
            raise ValueError('invalid CDR frame string')
        self.stamp_ns = sec * 1_000_000_000 + nano
        self.frame_id = bytes(self.body[12:11 + size]).decode('utf-8')
        offset = (12 + size + 7) // 8 * 8
        if offset + 20 > len(self.body):
            raise ValueError('truncated cloud metadata')
        self.timebase, self.point_num = struct.unpack_from(self.order + 'QI', self.body, offset)
        count = struct.unpack_from(self.order + 'I', self.body, offset + 16)[0]
        self.points_offset = offset + 20
        if count != self.point_num or len(self.body) < self.points_offset + max(0, count * 20 - 1):
            raise ValueError('cloud point count or payload length mismatch')

    def point(self, index):
        if not 0 <= index < self.point_num:
            raise IndexError(index)
        return struct.unpack_from(self.order + 'Ifff', self.body, self.points_offset + index * 20)
