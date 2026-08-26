package org.team100.lib.subsystems.se2;

import org.team100.lib.state.StateSE2;
import org.team100.lib.state.VelocityControlSE2;

public class MockSubsystemSE2 implements VelocitySubsystemSE2 {
    public VelocityControlSE2 m_setpoint;
    public VelocityControlSE2 m_recentSetpoint;
    public StateSE2 m_state;

    public MockSubsystemSE2(StateSE2 initial) {
        m_state = initial;
    }

    @Override
    public StateSE2 getState() {
        return m_state;
    }

    @Override
    public void stop() {
        // do nothing
    }

    @Override
    public void set(VelocityControlSE2 setpoint) {
        m_setpoint = setpoint;
        m_recentSetpoint = setpoint;
    }

}
