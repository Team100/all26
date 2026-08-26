package org.team100.lib.subsystems.se2.commands;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;

/**
 * Record the current pose at initialization time and hold that pose,
 * motionless, forever.
 */
public class HoldPosition extends MoveAndHold {
    private final PositionSubsystemSE2 m_subsystem;

    private StateSE2 m_state;

    public HoldPosition(PositionSubsystemSE2 subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // motionless at the current location
        m_state = new StateSE2(m_subsystem.getState().pose());
    }

    @Override
    public void execute() {
        m_subsystem.set(m_state.control());
        // m_subsystem.stop();
    }

    @Override
    public boolean isDone() {
        return true;
    }

    @Override
    public double toGo() {
        return 0;
    }

}
