package org.team100.lib.network;

import org.wpilib.networktables.ConnectionInfo;

public class NetworkUtil {
    public static String ciString(ConnectionInfo ci) {
        return String.format("%s %s %d %d %X",
                ci.remoteId, ci.remoteIp, ci.remotePort, ci.lastUpdate, ci.protocolVersion);
    }

}
