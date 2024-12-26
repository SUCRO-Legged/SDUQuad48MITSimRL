/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 * lcm-gen 1.5.0
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class spi_torque_t implements lcm.lcm.LCMEncodable
{
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

 
    public spi_torque_t()
    {
        tau_abad = new float[4];
        tau_hip = new float[4];
        tau_knee = new float[4];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xa847131dd865b527L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.spi_torque_t.class))
            return 0L;
 
        classes.add(lcmtypes.spi_torque_t.class);
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
            outs.writeFloat(this.tau_abad[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.tau_hip[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.tau_knee[a]); 
        }
 
    }
 
    public spi_torque_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public spi_torque_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.spi_torque_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.spi_torque_t o = new lcmtypes.spi_torque_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
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
 
    public lcmtypes.spi_torque_t copy()
    {
        lcmtypes.spi_torque_t outobj = new lcmtypes.spi_torque_t();
        outobj.tau_abad = new float[(int) 4];
        System.arraycopy(this.tau_abad, 0, outobj.tau_abad, 0, 4); 
        outobj.tau_hip = new float[(int) 4];
        System.arraycopy(this.tau_hip, 0, outobj.tau_hip, 0, 4); 
        outobj.tau_knee = new float[(int) 4];
        System.arraycopy(this.tau_knee, 0, outobj.tau_knee, 0, 4); 
        return outobj;
    }
 
}

