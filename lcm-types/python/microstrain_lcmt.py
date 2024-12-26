"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

from io import BytesIO
import struct

class microstrain_lcmt(object):

    __slots__ = ["quat", "rpy", "omega", "acc", "good_packets", "bad_packets"]

    __typenames__ = ["float", "float", "float", "float", "int64_t", "int64_t"]

    __dimensions__ = [[4], [3], [3], [3], None, None]

    def __init__(self):
        self.quat = [ 0.0 for dim0 in range(4) ]
        """ LCM Type: float[4] """
        self.rpy = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: float[3] """
        self.omega = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: float[3] """
        self.acc = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: float[3] """
        self.good_packets = 0
        """ LCM Type: int64_t """
        self.bad_packets = 0
        """ LCM Type: int64_t """

    def encode(self):
        buf = BytesIO()
        buf.write(microstrain_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>4f', *self.quat[:4]))
        buf.write(struct.pack('>3f', *self.rpy[:3]))
        buf.write(struct.pack('>3f', *self.omega[:3]))
        buf.write(struct.pack('>3f', *self.acc[:3]))
        buf.write(struct.pack(">qq", self.good_packets, self.bad_packets))

    @staticmethod
    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != microstrain_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return microstrain_lcmt._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = microstrain_lcmt()
        self.quat = struct.unpack('>4f', buf.read(16))
        self.rpy = struct.unpack('>3f', buf.read(12))
        self.omega = struct.unpack('>3f', buf.read(12))
        self.acc = struct.unpack('>3f', buf.read(12))
        self.good_packets, self.bad_packets = struct.unpack(">qq", buf.read(16))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if microstrain_lcmt in parents: return 0
        tmphash = (0x710a98f509c97d55) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if microstrain_lcmt._packed_fingerprint is None:
            microstrain_lcmt._packed_fingerprint = struct.pack(">Q", microstrain_lcmt._get_hash_recursive([]))
        return microstrain_lcmt._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", microstrain_lcmt._get_packed_fingerprint())[0]
