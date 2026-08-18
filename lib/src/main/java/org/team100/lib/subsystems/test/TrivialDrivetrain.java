package org.team100.lib.subsystems.test;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Executes desired velocity exactly.
 */
public class TrivialDrivetrain implements VelocitySubsystemSE2 {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private final ObjectCache<ModelSE2> m_stateCache;
    /** setpoint for the next timestep. used only by update(). */
    private VelocityControlSE2 m_setpoint;
    private ModelSE2 m_state;

    public TrivialDrivetrain(Pose2d initial) {
        m_setpoint = new VelocityControlSE2(0, 0, 0);
        m_state = new ModelSE2(initial);
        m_stateCache = Cache.of(this::update);
    }

    @Override
    public ModelSE2 getState() {
        return m_stateCache.get();
    }

    @Override
    public void stop() {
        m_setpoint = new VelocityControlSE2(0, 0, 0);
    }

    @Override
    public void set(VelocityControlSE2 setpoint) {
        m_setpoint = setpoint;
    }

    private ModelSE2 update() {
        m_state = new ModelSE2(
                m_setpoint.integrate(m_state.pose(), DT),
                m_setpoint.velocity());
        return m_state;
    }

}
