"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

from io import BytesIO
import struct

class SDUog_Parameter_t(object):

    __slots__ = ["timestamp", "position", "orientation", "num_ranges", "ranges", "name", "enabled", "vm_kp_roll", "vm_kp_pitch", "vm_kp_height", "vm_kd_roll", "vm_kd_pitch", "vm_kd_height", "s", "w", "vm_kp_fx", "vm_kp_fy", "vm_kp_fomega", "vm_kd_fx", "vm_kd_fy", "vm_kd_fomega", "sduog_value1", "sduog_value2", "sduog_value3", "sduog_value4", "sduog_value5", "sduog_value6"]

    __typenames__ = ["int64_t", "double", "double", "int32_t", "int16_t", "string", "boolean", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"]

    __dimensions__ = [None, [3], [4], None, ["num_ranges"], None, None, None, None, None, None, None, None, [6], [6], None, None, None, None, None, None, None, None, None, None, None, None]

    def __init__(self):
        self.timestamp = 0
        """ LCM Type: int64_t """
        self.position = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """
        self.orientation = [ 0.0 for dim0 in range(4) ]
        """ LCM Type: double[4] """
        self.num_ranges = 0
        """ LCM Type: int32_t """
        self.ranges = []
        """ LCM Type: int16_t[num_ranges] """
        self.name = ""
        """ LCM Type: string """
        self.enabled = False
        """ LCM Type: boolean """
        self.vm_kp_roll = 0.0
        """ LCM Type: double """
        self.vm_kp_pitch = 0.0
        """ LCM Type: double """
        self.vm_kp_height = 0.0
        """ LCM Type: double """
        self.vm_kd_roll = 0.0
        """ LCM Type: double """
        self.vm_kd_pitch = 0.0
        """ LCM Type: double """
        self.vm_kd_height = 0.0
        """ LCM Type: double """
        self.s = [ 0.0 for dim0 in range(6) ]
        """ LCM Type: double[6] """
        self.w = [ 0.0 for dim0 in range(6) ]
        """ LCM Type: double[6] """
        self.vm_kp_fx = 0.0
        """ LCM Type: double """
        self.vm_kp_fy = 0.0
        """ LCM Type: double """
        self.vm_kp_fomega = 0.0
        """ LCM Type: double """
        self.vm_kd_fx = 0.0
        """ LCM Type: double """
        self.vm_kd_fy = 0.0
        """ LCM Type: double """
        self.vm_kd_fomega = 0.0
        """ LCM Type: double """
        self.sduog_value1 = 0.0
        """ LCM Type: double """
        self.sduog_value2 = 0.0
        """ LCM Type: double """
        self.sduog_value3 = 0.0
        """ LCM Type: double """
        self.sduog_value4 = 0.0
        """ LCM Type: double """
        self.sduog_value5 = 0.0
        """ LCM Type: double """
        self.sduog_value6 = 0.0
        """ LCM Type: double """

    def encode(self):
        buf = BytesIO()
        buf.write(SDUog_Parameter_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.timestamp))
        buf.write(struct.pack('>3d', *self.position[:3]))
        buf.write(struct.pack('>4d', *self.orientation[:4]))
        buf.write(struct.pack(">i", self.num_ranges))
        buf.write(struct.pack('>%dh' % self.num_ranges, *self.ranges[:self.num_ranges]))
        __name_encoded = self.name.encode('utf-8')
        buf.write(struct.pack('>I', len(__name_encoded)+1))
        buf.write(__name_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">bdddddd", self.enabled, self.vm_kp_roll, self.vm_kp_pitch, self.vm_kp_height, self.vm_kd_roll, self.vm_kd_pitch, self.vm_kd_height))
        buf.write(struct.pack('>6d', *self.s[:6]))
        buf.write(struct.pack('>6d', *self.w[:6]))
        buf.write(struct.pack(">dddddddddddd", self.vm_kp_fx, self.vm_kp_fy, self.vm_kp_fomega, self.vm_kd_fx, self.vm_kd_fy, self.vm_kd_fomega, self.sduog_value1, self.sduog_value2, self.sduog_value3, self.sduog_value4, self.sduog_value5, self.sduog_value6))

    @staticmethod
    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != SDUog_Parameter_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return SDUog_Parameter_t._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = SDUog_Parameter_t()
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        self.position = struct.unpack('>3d', buf.read(24))
        self.orientation = struct.unpack('>4d', buf.read(32))
        self.num_ranges = struct.unpack(">i", buf.read(4))[0]
        self.ranges = struct.unpack('>%dh' % self.num_ranges, buf.read(self.num_ranges * 2))
        __name_len = struct.unpack('>I', buf.read(4))[0]
        self.name = buf.read(__name_len)[:-1].decode('utf-8', 'replace')
        self.enabled = bool(struct.unpack('b', buf.read(1))[0])
        self.vm_kp_roll, self.vm_kp_pitch, self.vm_kp_height, self.vm_kd_roll, self.vm_kd_pitch, self.vm_kd_height = struct.unpack(">dddddd", buf.read(48))
        self.s = struct.unpack('>6d', buf.read(48))
        self.w = struct.unpack('>6d', buf.read(48))
        self.vm_kp_fx, self.vm_kp_fy, self.vm_kp_fomega, self.vm_kd_fx, self.vm_kd_fy, self.vm_kd_fomega, self.sduog_value1, self.sduog_value2, self.sduog_value3, self.sduog_value4, self.sduog_value5, self.sduog_value6 = struct.unpack(">dddddddddddd", buf.read(96))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if SDUog_Parameter_t in parents: return 0
        tmphash = (0xb84ebdd84c9cfe38) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if SDUog_Parameter_t._packed_fingerprint is None:
            SDUog_Parameter_t._packed_fingerprint = struct.pack(">Q", SDUog_Parameter_t._get_hash_recursive([]))
        return SDUog_Parameter_t._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", SDUog_Parameter_t._get_packed_fingerprint())[0]

