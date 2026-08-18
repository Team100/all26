package org.team100.lib.network;

import edu.wpi.first.networktables.ConnectionInfo;

public class NetworkUtil {
    public static String ciString(ConnectionInfo ci) {
        return String.format("%s %s %d %d %X",
                ci.remote_id, ci.remote_ip, ci.remote_port, ci.last_update, ci.protocol_version);
    }

}
