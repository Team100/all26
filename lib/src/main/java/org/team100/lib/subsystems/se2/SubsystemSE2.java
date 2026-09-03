package org.team100.lib.subsystems.se2;

import org.team100.lib.state.StateSE2;

import org.wpilib.command2.Subsystem;

/**
 * A subsystem that represents planar rigid-body transforms, i.e. the SE(2) Lie
 * group.
 */
public interface SubsystemSE2 extends Subsystem {
    /**
     * State for the current Takt.
     * 
     * Because this measurement is often used for control (e.g. point from the
     * current position to some target), it is important to minimize noise in the
     * measurement.
     */
    StateSE2 getState();

    /** Passthrough to motor stop. This is not "hold position", it is "disable". */
    void stop();

}
