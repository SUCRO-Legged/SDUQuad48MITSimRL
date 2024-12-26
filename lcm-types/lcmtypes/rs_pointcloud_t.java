/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 * lcm-gen 1.5.0
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class rs_pointcloud_t implements lcm.lcm.LCMEncodable
{
    /**
     * LCM Type: double[5001][3]
     */
    public double pointlist[][];

 
    public rs_pointcloud_t()
    {
        pointlist = new double[5001][3];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x960bbc9070e3509eL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.rs_pointcloud_t.class))
            return 0L;
 
        classes.add(lcmtypes.rs_pointcloud_t.class);
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
        for (int a = 0; a < 5001; a++) {
            for (int b = 0; b < 3; b++) {
                outs.writeDouble(this.pointlist[a][b]); 
            }
        }
 
    }
 
    public rs_pointcloud_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public rs_pointcloud_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.rs_pointcloud_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.rs_pointcloud_t o = new lcmtypes.rs_pointcloud_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.pointlist = new double[(int) 5001][(int) 3];
        for (int a = 0; a < 5001; a++) {
            for (int b = 0; b < 3; b++) {
                this.pointlist[a][b] = ins.readDouble();
            }
        }
 
    }
 
    public lcmtypes.rs_pointcloud_t copy()
    {
        lcmtypes.rs_pointcloud_t outobj = new lcmtypes.rs_pointcloud_t();
        outobj.pointlist = new double[(int) 5001][(int) 3];
        for (int a = 0; a < 5001; a++) {
            System.arraycopy(this.pointlist[a], 0, outobj.pointlist[a], 0, 3);        }
 
        return outobj;
    }
 
}

