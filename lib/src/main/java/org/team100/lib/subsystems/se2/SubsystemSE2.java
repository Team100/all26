package org.team100.lib.subsystems.se2;

import org.team100.lib.state.ModelSE2;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** A planar subsystem */
public interface SubsystemSE2 extends Subsystem {
    /** State for the current Takt. */
    ModelSE2 getState();

    /** Passthrough to motor stop. This is not "hold position", it is "disable". */
    void stop();

}
