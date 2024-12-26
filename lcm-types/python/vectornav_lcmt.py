"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

from io import BytesIO
import struct

class vectornav_lcmt(object):

    __slots__ = ["q", "w", "a"]

    __typenames__ = ["float", "float", "float"]

    __dimensions__ = [[4], [3], [3]]

    def __init__(self):
        self.q = [ 0.0 for dim0 in range(4) ]
        """ LCM Type: float[4] """
        self.w = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: float[3] """
        self.a = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: float[3] """

    def encode(self):
        buf = BytesIO()
        buf.write(vectornav_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>4f', *self.q[:4]))
        buf.write(struct.pack('>3f', *self.w[:3]))
        buf.write(struct.pack('>3f', *self.a[:3]))

    @staticmethod
    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != vectornav_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return vectornav_lcmt._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = vectornav_lcmt()
        self.q = struct.unpack('>4f', buf.read(16))
        self.w = struct.unpack('>3f', buf.read(12))
        self.a = struct.unpack('>3f', buf.read(12))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if vectornav_lcmt in parents: return 0
        tmphash = (0xf57906decbf7ebdc) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if vectornav_lcmt._packed_fingerprint is None:
            vectornav_lcmt._packed_fingerprint = struct.pack(">Q", vectornav_lcmt._get_hash_recursive([]))
        return vectornav_lcmt._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", vectornav_lcmt._get_packed_fingerprint())[0]

