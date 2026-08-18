package org.team100.lib.localization;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.Struct;

public class BlipWithCornersStruct implements Struct<BlipWithCorners> {

    @Override
    public Class<BlipWithCorners> getTypeClass() {
        return BlipWithCorners.class;
    }

    @Override
    public String getTypeName() {
        return "BlipWithCorners";
    }

    @Override
    public int getSize() {
        return kSizeInt64 + kSizeInt32
                + kSizeFloat + kSizeFloat
                + kSizeFloat + kSizeFloat
                + kSizeFloat + kSizeFloat
                + kSizeFloat + kSizeFloat
                + Transform3d.struct.getSize();
    }

    @Override
    public String getSchema() {
        return "int64 timestamp; int32 id; "
                + "float x0; float y0; "
                + "float x1; float y1; "
                + "float x2; float y2; "
                + "float x3; float y3; "
                + "Transform3d pose";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] { Transform3d.struct };
    }

    @Override
    public BlipWithCorners unpack(ByteBuffer bb) {
        long timestamp = bb.getLong();
        int id = bb.getInt();
        float x0 = bb.getFloat();
        float y0 = bb.getFloat();
        float x1 = bb.getFloat();
        float y1 = bb.getFloat();
        float x2 = bb.getFloat();
        float y2 = bb.getFloat();
        float x3 = bb.getFloat();
        float y3 = bb.getFloat();
        Transform3d pose = Transform3d.struct.unpack(bb);
        return new BlipWithCorners(
                timestamp, id, x0, y0, x1, y1, x2, y2, x3, y3, pose);
    }

    @Override
    public void pack(ByteBuffer bb, BlipWithCorners value) {
        bb.putLong(value.getTimestamp());
        bb.putInt(value.getId());
        bb.putFloat(value.getX0());
        bb.putFloat(value.getY0());
        bb.putFloat(value.getX1());
        bb.putFloat(value.getY1());
        bb.putFloat(value.getX2());
        bb.putFloat(value.getY2());
        bb.putFloat(value.getX3());
        bb.putFloat(value.getY3());
        Transform3d.struct.pack(bb, value.getRawPose());
    }

}
