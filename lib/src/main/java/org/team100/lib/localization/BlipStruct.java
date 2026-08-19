package org.team100.lib.localization;

import java.nio.ByteBuffer;

import org.wpilib.math.geometry.Transform3d;
import org.wpilib.util.struct.Struct;

public class BlipStruct implements Struct<Blip> {

    @Override
    public Class<Blip> getTypeClass() {
        return Blip.class;
    }

    @Override
    public String getTypeName() {
        return "Blip";
    }

    @Override
    public int getSize() {
        return INT64_SIZE + INT32_SIZE + Transform3d.struct.getSize();
    }

    @Override
    public String getSchema() {
        return "int64 timestamp; int32 id; Transform3d pose";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] { Transform3d.struct };
    }

    @Override
    public Blip unpack(ByteBuffer bb) {
        long timestamp = bb.getLong();
        int id = bb.getInt();
        Transform3d pose = Transform3d.struct.unpack(bb);
        return new Blip(timestamp, id, pose);
    }

    @Override
    public void pack(ByteBuffer bb, Blip value) {
        bb.putLong(value.getTimestamp());
        bb.putInt(value.getId());
        Transform3d.struct.pack(bb, value.getRawPose());
    }

}
