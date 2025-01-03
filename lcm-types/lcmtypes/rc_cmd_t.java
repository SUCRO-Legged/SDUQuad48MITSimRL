/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 * lcm-gen 1.5.0
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class rc_cmd_t implements lcm.lcm.LCMEncodable
{
    public float linearVelX;

    public float linearVelY;

    public float linearVelZ;

    public float angularVelX;

    public float angularVelY;

    public float angularVelZ;

    public float roll;

    public float pitch;

    public float yaw;

    public float bodyHeight;

    public float stepHeight;

    public int mode;

    public int gait;

    public int update;

 
    public rc_cmd_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xc8177d4edc402ba4L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.rc_cmd_t.class))
            return 0L;
 
        classes.add(lcmtypes.rc_cmd_t.class);
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
        outs.writeFloat(this.linearVelX); 
 
        outs.writeFloat(this.linearVelY); 
 
        outs.writeFloat(this.linearVelZ); 
 
        outs.writeFloat(this.angularVelX); 
 
        outs.writeFloat(this.angularVelY); 
 
        outs.writeFloat(this.angularVelZ); 
 
        outs.writeFloat(this.roll); 
 
        outs.writeFloat(this.pitch); 
 
        outs.writeFloat(this.yaw); 
 
        outs.writeFloat(this.bodyHeight); 
 
        outs.writeFloat(this.stepHeight); 
 
        outs.writeInt(this.mode); 
 
        outs.writeInt(this.gait); 
 
        outs.writeInt(this.update); 
 
    }
 
    public rc_cmd_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public rc_cmd_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.rc_cmd_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.rc_cmd_t o = new lcmtypes.rc_cmd_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.linearVelX = ins.readFloat();
 
        this.linearVelY = ins.readFloat();
 
        this.linearVelZ = ins.readFloat();
 
        this.angularVelX = ins.readFloat();
 
        this.angularVelY = ins.readFloat();
 
        this.angularVelZ = ins.readFloat();
 
        this.roll = ins.readFloat();
 
        this.pitch = ins.readFloat();
 
        this.yaw = ins.readFloat();
 
        this.bodyHeight = ins.readFloat();
 
        this.stepHeight = ins.readFloat();
 
        this.mode = ins.readInt();
 
        this.gait = ins.readInt();
 
        this.update = ins.readInt();
 
    }
 
    public lcmtypes.rc_cmd_t copy()
    {
        lcmtypes.rc_cmd_t outobj = new lcmtypes.rc_cmd_t();
        outobj.linearVelX = this.linearVelX;
 
        outobj.linearVelY = this.linearVelY;
 
        outobj.linearVelZ = this.linearVelZ;
 
        outobj.angularVelX = this.angularVelX;
 
        outobj.angularVelY = this.angularVelY;
 
        outobj.angularVelZ = this.angularVelZ;
 
        outobj.roll = this.roll;
 
        outobj.pitch = this.pitch;
 
        outobj.yaw = this.yaw;
 
        outobj.bodyHeight = this.bodyHeight;
 
        outobj.stepHeight = this.stepHeight;
 
        outobj.mode = this.mode;
 
        outobj.gait = this.gait;
 
        outobj.update = this.update;
 
        return outobj;
    }
 
}

