"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

from io import BytesIO
import struct

class qp_controller_data_t(object):

    __slots__ = ["exit_flag", "nWSR", "cpu_time_microseconds", "xOpt", "p_des", "p_act", "v_des", "v_act", "O_err", "omegab_des", "omegab_act", "lbA", "ubA", "C_times_f", "b_control", "b_control_Opt", "active", "pfeet_des", "pfeet_act"]

    __typenames__ = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"]

    __dimensions__ = [None, None, None, [12], [3], [3], [3], [3], [3], [3], [3], [20], [20], [20], [6], [6], None, [12], [12]]

    def __init__(self):
        self.exit_flag = 0.0
        """ LCM Type: double """
        self.nWSR = 0.0
        """ LCM Type: double """
        self.cpu_time_microseconds = 0.0
        """ LCM Type: double """
        self.xOpt = [ 0.0 for dim0 in range(12) ]
        """ LCM Type: double[12] """
        self.p_des = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """
        self.p_act = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """
        self.v_des = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """
        self.v_act = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """
        self.O_err = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """
        self.omegab_des = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """
        self.omegab_act = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """
        self.lbA = [ 0.0 for dim0 in range(20) ]
        """ LCM Type: double[20] """
        self.ubA = [ 0.0 for dim0 in range(20) ]
        """ LCM Type: double[20] """
        self.C_times_f = [ 0.0 for dim0 in range(20) ]
        """ LCM Type: double[20] """
        self.b_control = [ 0.0 for dim0 in range(6) ]
        """ LCM Type: double[6] """
        self.b_control_Opt = [ 0.0 for dim0 in range(6) ]
        """ LCM Type: double[6] """
        self.active = 0.0
        """ LCM Type: double """
        self.pfeet_des = [ 0.0 for dim0 in range(12) ]
        """ LCM Type: double[12] """
        self.pfeet_act = [ 0.0 for dim0 in range(12) ]
        """ LCM Type: double[12] """

    def encode(self):
        buf = BytesIO()
        buf.write(qp_controller_data_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ddd", self.exit_flag, self.nWSR, self.cpu_time_microseconds))
        buf.write(struct.pack('>12d', *self.xOpt[:12]))
        buf.write(struct.pack('>3d', *self.p_des[:3]))
        buf.write(struct.pack('>3d', *self.p_act[:3]))
        buf.write(struct.pack('>3d', *self.v_des[:3]))
        buf.write(struct.pack('>3d', *self.v_act[:3]))
        buf.write(struct.pack('>3d', *self.O_err[:3]))
        buf.write(struct.pack('>3d', *self.omegab_des[:3]))
        buf.write(struct.pack('>3d', *self.omegab_act[:3]))
        buf.write(struct.pack('>20d', *self.lbA[:20]))
        buf.write(struct.pack('>20d', *self.ubA[:20]))
        buf.write(struct.pack('>20d', *self.C_times_f[:20]))
        buf.write(struct.pack('>6d', *self.b_control[:6]))
        buf.write(struct.pack('>6d', *self.b_control_Opt[:6]))
        buf.write(struct.pack(">d", self.active))
        buf.write(struct.pack('>12d', *self.pfeet_des[:12]))
        buf.write(struct.pack('>12d', *self.pfeet_act[:12]))

    @staticmethod
    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != qp_controller_data_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return qp_controller_data_t._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = qp_controller_data_t()
        self.exit_flag, self.nWSR, self.cpu_time_microseconds = struct.unpack(">ddd", buf.read(24))
        self.xOpt = struct.unpack('>12d', buf.read(96))
        self.p_des = struct.unpack('>3d', buf.read(24))
        self.p_act = struct.unpack('>3d', buf.read(24))
        self.v_des = struct.unpack('>3d', buf.read(24))
        self.v_act = struct.unpack('>3d', buf.read(24))
        self.O_err = struct.unpack('>3d', buf.read(24))
        self.omegab_des = struct.unpack('>3d', buf.read(24))
        self.omegab_act = struct.unpack('>3d', buf.read(24))
        self.lbA = struct.unpack('>20d', buf.read(160))
        self.ubA = struct.unpack('>20d', buf.read(160))
        self.C_times_f = struct.unpack('>20d', buf.read(160))
        self.b_control = struct.unpack('>6d', buf.read(48))
        self.b_control_Opt = struct.unpack('>6d', buf.read(48))
        self.active = struct.unpack(">d", buf.read(8))[0]
        self.pfeet_des = struct.unpack('>12d', buf.read(96))
        self.pfeet_act = struct.unpack('>12d', buf.read(96))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if qp_controller_data_t in parents: return 0
        tmphash = (0xa885737200dec04e) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if qp_controller_data_t._packed_fingerprint is None:
            qp_controller_data_t._packed_fingerprint = struct.pack(">Q", qp_controller_data_t._get_hash_recursive([]))
        return qp_controller_data_t._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", qp_controller_data_t._get_packed_fingerprint())[0]

