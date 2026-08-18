package org.team100.lib.targeting;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.struct.Struct;

public class TargetStruct implements Struct<Target> {

    @Override
    public Class<Target> getTypeClass() {
        return Target.class;
    }

    @Override
    public String getTypeName() {
        return "Target";
    }

    @Override
    public int getSize() {
        return kSizeInt64 + Rotation3d.struct.getSize();
    }

    @Override
    public String getSchema() {
        return "int64 timestamp; Rotation3d sight";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] { Rotation3d.struct };
    }

    @Override
    public Target unpack(ByteBuffer bb) {
        long timestamp = bb.getLong();
        Rotation3d sight = Rotation3d.struct.unpack(bb);
        return new Target(timestamp, sight);
    }

    @Override
    public void pack(ByteBuffer bb, Target value) {
        bb.putLong(value.getTimestamp());
        Rotation3d.struct.pack(bb, value.sight());
    }

}
