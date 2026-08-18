package org.team100.lib.reference.r1;

import java.util.function.Supplier;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.state.VelocityControlR1;

import edu.wpi.first.math.MathUtil;

public class VelocityProfileReferenceR1 implements VelocityReferenceR1 {
    private final DoubleLogger m_log_setpoint;
    private final BooleanLogger m_log_done;
    private final Supplier<VelocityProfileR1> m_profile;
    private final double m_tolerance;
    private Double m_goal;
    private double m_currentInstant;
    private VelocityControlR1 m_currentSetpoint;

    /**
     * @param parent
     * @param profile
     * @param tolerance used for profileDone()
     */
    public VelocityProfileReferenceR1(
            LoggerFactory parent,
            Supplier<VelocityProfileR1> profile,
            double tolerance) {
        LoggerFactory log = parent.type(this);
        m_log_setpoint = log.doubleLogger(Level.TRACE, "setpoint");
        m_log_done = log.booleanLogger(Level.TRACE, "done");
        m_profile = profile;
        m_tolerance = tolerance;
    }

    @Override
    public void setGoal(double goal) {
        m_goal = goal;
    }

    @Override
    public void init(double measurement) {
        m_currentInstant = Takt.get();
        m_currentSetpoint = advance(new VelocityControlR1(measurement, 0));
    }

    @Override
    public VelocityControlR1 get() {
        if (m_currentSetpoint == null)
            throw new IllegalStateException("Must init!");
        double t = Takt.get();
        if (t == m_currentInstant) {
            // Time hasn't passed since last time, so don't change anything.
            return m_currentSetpoint;
        }

        // Time has passed, make a new setpoint and return it.
        m_currentInstant = t;
        m_currentSetpoint = advance(m_currentSetpoint);
        m_log_setpoint.log(() -> m_currentSetpoint.v());
        return m_currentSetpoint;
    }

    @Override
    public boolean profileDone() {
        // the only way to tell if an incremental profile is done
        // is to compare the goal to the setpoint.
        boolean done = MathUtil.isNear(
                m_currentSetpoint.v(), m_goal, m_tolerance);
        m_log_done.log(() -> done);
        return done;
    }

    @Override
    public boolean valid() {
        return m_currentSetpoint != null;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////

    private VelocityControlR1 advance(VelocityControlR1 newCurrent) {
        if (m_goal == null)
            throw new IllegalStateException("goal must be set");
        return m_profile.get().calculate(
                TimedRobot100.LOOP_PERIOD_S, newCurrent, m_goal);
    }
}
