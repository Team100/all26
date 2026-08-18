package org.team100.lib.targeting;

import edu.wpi.first.math.geometry.Rotation3d;

public class Target {
    private final long timestamp;
    private final Rotation3d sight;

    public Target(long timestamp, Rotation3d sight) {
        this.timestamp = timestamp;
        this.sight = sight;
    }

    /**
     * Microseconds
     */
    public long getTimestamp() {
        return timestamp;
    }

    public Rotation3d sight() {
        return sight;
    }

    public static final TargetStruct struct = new TargetStruct();

}
