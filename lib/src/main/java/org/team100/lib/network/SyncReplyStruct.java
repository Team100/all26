package org.team100.lib.network;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;

public class SyncReplyStruct implements Struct<SyncReply> {

    @Override
    public Class<SyncReply> getTypeClass() {
        return SyncReply.class;
    }

    @Override
    public String getTypeName() {
        return "SyncReply";
    }

    @Override
    public int getSize() {
        return kSizeInt64 + kSizeInt64 + kSizeInt64;
    }

    @Override
    public String getSchema() {
        return "int64 org; int64 rec; int64 xmt";
    }

    @Override
    public SyncReply unpack(ByteBuffer bb) {
        long org = bb.getLong();
        long rec = bb.getLong();
        long xmt = bb.getLong();
        return new SyncReply(org, rec, xmt);
    }

    @Override
    public void pack(ByteBuffer bb, SyncReply value) {
        bb.putLong(value.org());
        bb.putLong(value.rec());
        bb.putLong(value.xmt());
    }
}
