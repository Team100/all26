package org.team100.lib.network;

import java.nio.ByteBuffer;

import org.wpilib.util.struct.Struct;

public class SyncRequestStruct implements Struct<SyncRequest> {

    @Override
    public Class<SyncRequest> getTypeClass() {
        return SyncRequest.class;
    }

    @Override
    public String getTypeName() {
        return "SyncRequest";
    }

    @Override
    public int getSize() {
        return INT64_SIZE;
    }

    @Override
    public String getSchema() {
        return "int64 org";
    }

    @Override
    public SyncRequest unpack(ByteBuffer bb) {
        long org = bb.getLong();
        return new SyncRequest(org);
    }

    @Override
    public void pack(ByteBuffer bb, SyncRequest value) {
        bb.putLong(value.org());
    }
}
