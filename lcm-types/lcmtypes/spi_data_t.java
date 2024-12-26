/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 * lcm-gen 1.5.0
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class spi_data_t implements lcm.lcm.LCMEncodable
{
    /**
     * LCM Type: float[4]
     */
    public float q_abad[];

    /**
     * LCM Type: float[4]
     */
    public float q_hip[];

    /**
     * LCM Type: float[4]
     */
    public float q_knee[];

    /**
     * LCM Type: float[4]
     */
    public float qd_abad[];

    /**
     * LCM Type: float[4]
     */
    public float qd_hip[];

    /**
     * LCM Type: float[4]
     */
    public float qd_knee[];

    /**
     * LCM Type: int32_t[4]
     */
    public int flags[];

    public int spi_driver_status;

    /**
     * LCM Type: float[4]
     */
    public float tau_abad[];

    /**
     * LCM Type: float[4]
     */
    public float tau_hip[];

    /**
     * LCM Type: float[4]
     */
    public float tau_knee[];

 
    public spi_data_t()
    {
        q_abad = new float[4];
        q_hip = new float[4];
        q_knee = new float[4];
        qd_abad = new float[4];
        qd_hip = new float[4];
        qd_knee = new float[4];
        flags = new int[4];
        tau_abad = new float[4];
        tau_hip = new float[4];
        tau_knee = new float[4];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x524a32c92be00b70L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.spi_data_t.class))
            return 0L;
 
        classes.add(lcmtypes.spi_data_t.class);
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
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.q_abad[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.q_hip[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.q_knee[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.qd_abad[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.qd_hip[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.qd_knee[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeInt(this.flags[a]); 
        }
 
        outs.writeInt(this.spi_driver_status); 
 
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.tau_abad[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.tau_hip[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.tau_knee[a]); 
        }
 
    }
 
    public spi_data_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public spi_data_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.spi_data_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.spi_data_t o = new lcmtypes.spi_data_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.q_abad = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.q_abad[a] = ins.readFloat();
        }
 
        this.q_hip = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.q_hip[a] = ins.readFloat();
        }
 
        this.q_knee = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.q_knee[a] = ins.readFloat();
        }
 
        this.qd_abad = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.qd_abad[a] = ins.readFloat();
        }
 
        this.qd_hip = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.qd_hip[a] = ins.readFloat();
        }
 
        this.qd_knee = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.qd_knee[a] = ins.readFloat();
        }
 
        this.flags = new int[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.flags[a] = ins.readInt();
        }
 
        this.spi_driver_status = ins.readInt();
 
        this.tau_abad = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.tau_abad[a] = ins.readFloat();
        }
 
        this.tau_hip = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.tau_hip[a] = ins.readFloat();
        }
 
        this.tau_knee = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.tau_knee[a] = ins.readFloat();
        }
 
    }
 
    public lcmtypes.spi_data_t copy()
    {
        lcmtypes.spi_data_t outobj = new lcmtypes.spi_data_t();
        outobj.q_abad = new float[(int) 4];
        System.arraycopy(this.q_abad, 0, outobj.q_abad, 0, 4); 
        outobj.q_hip = new float[(int) 4];
        System.arraycopy(this.q_hip, 0, outobj.q_hip, 0, 4); 
        outobj.q_knee = new float[(int) 4];
        System.arraycopy(this.q_knee, 0, outobj.q_knee, 0, 4); 
        outobj.qd_abad = new float[(int) 4];
        System.arraycopy(this.qd_abad, 0, outobj.qd_abad, 0, 4); 
        outobj.qd_hip = new float[(int) 4];
        System.arraycopy(this.qd_hip, 0, outobj.qd_hip, 0, 4); 
        outobj.qd_knee = new float[(int) 4];
        System.arraycopy(this.qd_knee, 0, outobj.qd_knee, 0, 4); 
        outobj.flags = new int[(int) 4];
        System.arraycopy(this.flags, 0, outobj.flags, 0, 4); 
        outobj.spi_driver_status = this.spi_driver_status;
 
        outobj.tau_abad = new float[(int) 4];
        System.arraycopy(this.tau_abad, 0, outobj.tau_abad, 0, 4); 
        outobj.tau_hip = new float[(int) 4];
        System.arraycopy(this.tau_hip, 0, outobj.tau_hip, 0, 4); 
        outobj.tau_knee = new float[(int) 4];
        System.arraycopy(this.tau_knee, 0, outobj.tau_knee, 0, 4); 
        return outobj;
    }
 
}

