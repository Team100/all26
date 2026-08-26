package org.team100.lib.reference.rn;

import org.team100.lib.subsystems.rn.PositionSubsystemRn;
import org.wpilib.math.util.Num;

/**
 * Actuates a positional N-DOF subsystem.
 * 
 * The lifespan of this object is intended to be a single "playback" of a
 * trajectory, so create it in Command.initialize().
 * 
 * TODO: add "toGo"
 * 
 */
public class PositionReferenceControllerRn<N extends Num> {
    private final PositionSubsystemRn<N> m_subsystem;
    private final ReferenceRn<N> m_reference;

    public PositionReferenceControllerRn(
            PositionSubsystemRn<N> subsystem,
            ReferenceRn<N> reference) {
        m_subsystem = subsystem;
        m_reference = reference;
        m_reference.init();
    }

    /**
     * This should be called in Command.execute().
     * 
     * TODO: add an onboard controller using "current()"
     */
    public void execute() {
        m_subsystem.setRn(m_reference.next());
    }

    public boolean isDone() {
        return m_reference.done();
    }
}
