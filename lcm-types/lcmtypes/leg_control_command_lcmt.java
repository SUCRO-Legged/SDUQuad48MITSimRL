/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 * lcm-gen 1.5.0
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class leg_control_command_lcmt implements lcm.lcm.LCMEncodable
{
    /**
     * LCM Type: float[12]
     */
    public float tau_ff[];

    /**
     * LCM Type: float[12]
     */
    public float f_ff[];

    /**
     * LCM Type: float[12]
     */
    public float q_des[];

    /**
     * LCM Type: float[12]
     */
    public float qd_des[];

    /**
     * LCM Type: float[12]
     */
    public float p_des[];

    /**
     * LCM Type: float[12]
     */
    public float v_des[];

    /**
     * LCM Type: float[12]
     */
    public float kp_cartesian[];

    /**
     * LCM Type: float[12]
     */
    public float kd_cartesian[];

    /**
     * LCM Type: float[12]
     */
    public float kp_joint[];

    /**
     * LCM Type: float[12]
     */
    public float kd_joint[];

 
    public leg_control_command_lcmt()
    {
        tau_ff = new float[12];
        f_ff = new float[12];
        q_des = new float[12];
        qd_des = new float[12];
        p_des = new float[12];
        v_des = new float[12];
        kp_cartesian = new float[12];
        kd_cartesian = new float[12];
        kp_joint = new float[12];
        kd_joint = new float[12];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x93bfbc95a989bb67L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.leg_control_command_lcmt.class))
            return 0L;
 
        classes.add(lcmtypes.leg_control_command_lcmt.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.tau_ff[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.f_ff[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.q_des[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.qd_des[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.p_des[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.v_des[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.kp_cartesian[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.kd_cartesian[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.kp_joint[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeFloat(this.kd_joint[a]); 
        }
 
    }
 
    public leg_control_command_lcmt(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public leg_control_command_lcmt(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.leg_control_command_lcmt _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.leg_control_command_lcmt o = new lcmtypes.leg_control_command_lcmt();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.tau_ff = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.tau_ff[a] = ins.readFloat();
        }
 
        this.f_ff = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.f_ff[a] = ins.readFloat();
        }
 
        this.q_des = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.q_des[a] = ins.readFloat();
        }
 
        this.qd_des = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.qd_des[a] = ins.readFloat();
        }
 
        this.p_des = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.p_des[a] = ins.readFloat();
        }
 
        this.v_des = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.v_des[a] = ins.readFloat();
        }
 
        this.kp_cartesian = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.kp_cartesian[a] = ins.readFloat();
        }
 
        this.kd_cartesian = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.kd_cartesian[a] = ins.readFloat();
        }
 
        this.kp_joint = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.kp_joint[a] = ins.readFloat();
        }
 
        this.kd_joint = new float[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.kd_joint[a] = ins.readFloat();
        }
 
    }
 
    public lcmtypes.leg_control_command_lcmt copy()
    {
        lcmtypes.leg_control_command_lcmt outobj = new lcmtypes.leg_control_command_lcmt();
        outobj.tau_ff = new float[(int) 12];
        System.arraycopy(this.tau_ff, 0, outobj.tau_ff, 0, 12); 
        outobj.f_ff = new float[(int) 12];
        System.arraycopy(this.f_ff, 0, outobj.f_ff, 0, 12); 
        outobj.q_des = new float[(int) 12];
        System.arraycopy(this.q_des, 0, outobj.q_des, 0, 12); 
        outobj.qd_des = new float[(int) 12];
        System.arraycopy(this.qd_des, 0, outobj.qd_des, 0, 12); 
        outobj.p_des = new float[(int) 12];
        System.arraycopy(this.p_des, 0, outobj.p_des, 0, 12); 
        outobj.v_des = new float[(int) 12];
        System.arraycopy(this.v_des, 0, outobj.v_des, 0, 12); 
        outobj.kp_cartesian = new float[(int) 12];
        System.arraycopy(this.kp_cartesian, 0, outobj.kp_cartesian, 0, 12); 
        outobj.kd_cartesian = new float[(int) 12];
        System.arraycopy(this.kd_cartesian, 0, outobj.kd_cartesian, 0, 12); 
        outobj.kp_joint = new float[(int) 12];
        System.arraycopy(this.kp_joint, 0, outobj.kp_joint, 0, 12); 
        outobj.kd_joint = new float[(int) 12];
        System.arraycopy(this.kd_joint, 0, outobj.kd_joint, 0, 12); 
        return outobj;
    }
 
}

