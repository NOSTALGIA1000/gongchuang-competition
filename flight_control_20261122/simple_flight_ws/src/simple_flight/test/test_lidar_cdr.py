import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simple_flight.lidar_cdr import CloudView


def fixture(frame='livox_frame', count=3):
    data = bytearray(struct.pack('<iII', 123, 456, len(frame) + 1))
    data += frame.encode() + b'\0'
    data += b'\0' * (-len(data) % 8)
    data += struct.pack('<QIB3sI', 123000000456, count, 0, b'\0' * 3, count)
    for i in range(count):
        data += b'\0' * (-len(data) % 4)
        data += struct.pack('<IfffBBB', i * 1000, float(i), 2., 3., 1, 0, 0)
    return b'\0\1\0\0' + data


class CloudCdrTests(unittest.TestCase):
    def test_padding_and_last_point(self):
        for frame in ['a', 'ab', 'livox_frame', 'another_frame_id']:
            view = CloudView(fixture(frame))
            self.assertEqual(view.frame_id, frame)
            self.assertEqual(view.stamp_ns, 123000000456)
            self.assertEqual(view.timebase, view.stamp_ns)
            self.assertEqual(view.point_num, 3)
            self.assertEqual(view.point(2), (2000, 2., 2., 3.))

    def test_truncated_payload_rejected(self):
        with self.assertRaises(ValueError):
            CloudView(fixture()[:-1])

    def test_unsupported_encoding_rejected(self):
        with self.assertRaises(ValueError):
            CloudView(b'\0\3\0\0' + fixture()[4:])
