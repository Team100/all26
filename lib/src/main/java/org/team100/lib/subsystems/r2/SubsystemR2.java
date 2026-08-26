package org.team100.lib.subsystems.r2;

import org.team100.lib.state.StateR2;
import org.wpilib.command2.Subsystem;

/** A planar subsystem for position only, not rotation. */
public interface SubsystemR2 extends Subsystem {
    /** State for the current Takt. */
    StateR2 getState();

    /** Passthrough to motor stop. This is not "hold position", it is "disable". */
    void stop();
}
