package org.team100.lib.servo;

import org.team100.lib.dynamics.r.RDynamics;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;

import edu.wpi.first.math.MathUtil;

/**
 * Common elements of angular position servos.
 * 
 * This uses the "short way" between measurement and goal or setpoint, unless
 * that path exceeds the mechanism bounds. In that case, it takes the "long
 * way".
 */
public abstract class AngularPositionServoImpl implements AngularPositionServo {
    private static final boolean DEBUG = false;
    protected final RotaryMechanism m_mechanism;
    protected final RDynamics m_dynamics;
    private final ReferenceR1 m_ref;
    private final double m_positionTolerance;
    private final double m_velocityTolerance;

    private final BooleanLogger m_log_atGoal;
    private final DoubleLogger m_log_goal;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_acceleration;
    private final BooleanLogger m_log_at_setpoint;
    private final BooleanLogger m_log_profile_done;
    private final BooleanLogger m_log_at_goal;

    /**
     * Goal is "unwrapped" i.e. it's it's [-inf, inf], not [-pi,pi]
     */
    private StateR1 m_unwrappedGoal = new StateR1(0, 0);
    /**
     * Setpoint is "unwrapped" i.e. it's [-inf, inf], not [-pi,pi]
     * This is written when it is the setpoint for the "next" time step, i.e. the
     * one we use for feedforward, and the next cycle it is read as the "current"
     * setpoint.
     */
    ControlR1 m_nextUnwrappedSetpoint = null;

    /**
     * When the goal or setpoint is in an inaccessible zone, we hold position, so
     * there is a setpoint, but it's not the one the client asked for.
     */
    boolean m_validSetpoint;

    protected AngularPositionServoImpl(
            LoggerFactory parent,
            RotaryMechanism mechanism,
            RDynamics dynamics,
            ReferenceR1 ref,
            double xtolerance,
            double vtolerance) {
        m_mechanism = mechanism;
        m_dynamics = dynamics;
        m_ref = ref;
        m_positionTolerance = xtolerance;
        m_velocityTolerance = vtolerance;
        LoggerFactory log = parent.type(this);
        m_log_atGoal = log.booleanLogger(Level.TRACE, "at goal");
        m_log_goal = log.doubleLogger(Level.TRACE, "goal (rad)");
        m_log_position = log.doubleLogger(Level.COMP, "position (rad)");
        m_log_velocity = log.doubleLogger(Level.COMP, "velocity (rad_s)");
        m_log_acceleration = log.doubleLogger(Level.COMP, "accel (rad_s2)");
        m_log_at_setpoint = log.booleanLogger(Level.TRACE, "at setpoint");
        m_log_profile_done = log.booleanLogger(Level.TRACE, "profile done");
        m_log_at_goal = log.booleanLogger(Level.TRACE, "at goal");
        LogPoller.register(this::log);
    }

    abstract void actuate(SetpointsR1 wrappedSetpoints);

    @Override
    public void reset() {
        m_nextUnwrappedSetpoint = null;
        ControlR1 measurement = new ControlR1(getWrappedPositionRad(), 0);
        m_ref.setGoal(measurement.state());
        m_ref.init(measurement.state());
    }

    @Override
    public void setVoltage(double v) {
        m_mechanism.setVoltage(v);
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double x) {
        m_mechanism.setUnwrappedEncoderPositionRad(x);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        m_unwrappedGoal = null;
        m_nextUnwrappedSetpoint = null;
        m_mechanism.setDutyCycle(dutyCycle);
    }

    @Override
    public void setPositionDirect(double wrappedGoalRad, double velocityRad_S) {
        if (DEBUG)
            System.out.printf("setPositionDirect %6.3f %6.3f\n", wrappedGoalRad, velocityRad_S);
        m_log_velocity.log(() -> velocityRad_S);
        // make sure the reference gets reinitialized if required later
        m_unwrappedGoal = null;
        m_validSetpoint = true;

        double unwrappedMeasurement = m_mechanism.getUnwrappedPositionRad();
        if (DEBUG)
            System.out.printf("unwrappedMeasurement %6.3f\n", unwrappedMeasurement);
        double dx = MathUtil.angleModulus(wrappedGoalRad - unwrappedMeasurement);
        double unwrappedGoalX = unwrappedMeasurement + dx;
        if (DEBUG)
            System.out.printf("unwrappedGoalX %6.3f\n", unwrappedGoalX);
        if (dx > 0) {
            if (DEBUG)
                System.out.println("short way is positive");
            if (unwrappedGoalX > m_mechanism.getMaxPositionRad()) {
                if (DEBUG)
                    System.out.println("short way is beyond the limit");
                unwrappedGoalX = unwrappedGoalX - 2 * Math.PI;
                if (unwrappedGoalX >= m_mechanism.getMinPositionRad()) {
                    if (DEBUG)
                        System.out.println("use a profile to go around");
                    actuateProfiledImpl(unwrappedGoalX);
                    return;
                } else {
                    if (DEBUG)
                        System.out.println("setpoint is inaccessible, hold position.");
                    m_nextUnwrappedSetpoint = m_mechanism.getUnwrappedMeasurement().control();
                    actuate(new SetpointsR1(m_nextUnwrappedSetpoint, m_nextUnwrappedSetpoint));
                    m_validSetpoint = false;
                    return;
                }
            }
        } else {
            if (DEBUG)
                System.out.println("short way is negative");
            if (unwrappedGoalX < m_mechanism.getMinPositionRad()) {
                if (DEBUG)
                    System.out.println("short way is beyond the limit");
                unwrappedGoalX = unwrappedGoalX + 2 * Math.PI;
                if (unwrappedGoalX <= m_mechanism.getMaxPositionRad()) {
                    if (DEBUG)
                        System.out.println("use a profile to go around");
                    actuateProfiledImpl(unwrappedGoalX);
                    return;
                } else {
                    if (DEBUG)
                        System.out.println("setpoint is inaccessible; hold position.");
                    m_nextUnwrappedSetpoint = m_mechanism.getUnwrappedMeasurement().control();
                    actuate(new SetpointsR1(m_nextUnwrappedSetpoint, m_nextUnwrappedSetpoint));
                    m_validSetpoint = false;
                    return;
                }
            }
        }
        // this was the setpoint from the previous iteration
        ControlR1 currentUnwrappedSetpoint = m_nextUnwrappedSetpoint;
        // use the feedforward velocity
        m_nextUnwrappedSetpoint = new ControlR1(unwrappedGoalX, velocityRad_S);
        if (currentUnwrappedSetpoint == null)
            currentUnwrappedSetpoint = m_nextUnwrappedSetpoint;
        actuate(new SetpointsR1(currentUnwrappedSetpoint, m_nextUnwrappedSetpoint));
    }

    @Override
    public void setPositionProfiled(double wrappedGoalRad) {
        m_log_goal.log(() -> wrappedGoalRad);
        m_validSetpoint = true;
        double unwrappedMeasurement = m_mechanism.getUnwrappedPositionRad();
        double dx = MathUtil.angleModulus(wrappedGoalRad - unwrappedMeasurement);
        double unwrappedGoalX = unwrappedMeasurement + dx;
        if (dx > 0) {
            if (DEBUG)
                System.out.println("short way is positive");
            if (unwrappedGoalX > m_mechanism.getMaxPositionRad()) {
                if (DEBUG)
                    System.out.println("goal is beyond the mechanism range, go the other way.");
                unwrappedGoalX = unwrappedGoalX - 2 * Math.PI;
                if (unwrappedGoalX < m_mechanism.getMinPositionRad()) {
                    if (DEBUG)
                        System.out.println("the goal is inaccessible, just hold position.");
                    unwrappedGoalX = unwrappedMeasurement;
                    m_validSetpoint = false;
                }
            }
        } else {
            if (DEBUG)
                System.out.println("short way is negative");
            if (unwrappedGoalX < m_mechanism.getMinPositionRad()) {
                if (DEBUG)
                    System.out.println("goal is too far, try an equivalent goal");
                unwrappedGoalX = unwrappedGoalX + 2 * Math.PI;
                if (unwrappedGoalX > m_mechanism.getMaxPositionRad()) {
                    if (DEBUG)
                        System.out.println("the goal is inaccessible, just hold position.");
                    unwrappedGoalX = unwrappedMeasurement;
                    m_validSetpoint = false;
                }
            }
        }
        actuateProfiledImpl(unwrappedGoalX);
    }

    @Override
    public void setVelocity(double rad_S) {
        m_mechanism.setVelocity(rad_S, 0);
    }

    @Override
    public void play(double freq) {
        m_mechanism.play(freq);
    }

    @Override
    public void actuateWithProfile(double unwrappedGoalX) {
        m_validSetpoint = true;
        actuateProfiledImpl(unwrappedGoalX);
    }

    @Override
    public void actuateDirect(double unwrappedSetpoint) {
        m_validSetpoint = true;
        m_unwrappedGoal = null;
        m_nextUnwrappedSetpoint = null;
        ControlR1 c = new ControlR1(unwrappedSetpoint);
        actuate(new SetpointsR1(c, c));
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_mechanism.setTorqueLimit(torqueNm);
    }

    /**
     * @return the absolute 1:1 position of the mechanism in [-pi, pi]
     */
    @Override
    public double getWrappedPositionRad() {
        return m_mechanism.getWrappedPositionRad();
    }

    @Override
    public double getUnwrappedPositionRad() {
        return m_mechanism.getUnwrappedPositionRad();
    }

    @Override
    public double getVelocity() {
        return m_mechanism.getVelocityRad_S();
    }

    @Override
    public double getAcceleration() {
        return m_mechanism.getAccelerationRad_S2();
    }

    @Override
    public StateR1 getUnwrappedGoal() {
        return m_unwrappedGoal;
    }

    @Override
    public boolean validSetpoint() {
        return m_validSetpoint;
    }

    /**
     * Compares robotPeriodic-updated measurements to the setpoint,
     * so you need to know when the setpoint was updated: is it for the
     * current Takt time, or the next step?
     */
    @Override
    public boolean atSetpoint() {
        if (!m_validSetpoint) {
            if (DEBUG)
                System.out.println("NO SETPOINT");
            return false;
        }
        if (m_nextUnwrappedSetpoint == null) {
            if (DEBUG)
                System.out.println("NO NEXT SETPOINT");
            return false;
        }
        double positionError = MathUtil.angleModulus(m_nextUnwrappedSetpoint.x() - m_mechanism.getWrappedPositionRad());
        double velocityError = m_nextUnwrappedSetpoint.v() - m_mechanism.getVelocityRad_S();
        return Math.abs(positionError) < m_positionTolerance
                && Math.abs(velocityError) < m_velocityTolerance;
    }

    @Override
    public boolean profileDone() {
        if (m_unwrappedGoal == null) {
            // if there's no profile, it's always done.
            if (DEBUG)
                System.out.println("no profile, always done");
            return true;
        }
        boolean profileDone = m_ref.profileDone();
        if (DEBUG)
            System.out.printf("profile done %b\n", profileDone);
        return profileDone;
    }

    @Override
    public boolean atGoal() {
        boolean b = atSetpoint() && profileDone();
        if (DEBUG)
            System.out.printf("at goal %b\n", b);
        return b;
    }

    @Override
    public void stop() {
        m_unwrappedGoal = null;
        m_nextUnwrappedSetpoint = null;
        m_mechanism.stop();
    }

    @Override
    public void close() {
        m_mechanism.close();
    }

    private void log() {
        m_log_atGoal.log(() -> atGoal());
        m_log_position.log(() -> getUnwrappedPositionRad());
        m_log_velocity.log(() -> getVelocity());
        m_log_acceleration.log(() -> getAcceleration());
        m_log_at_setpoint.log(() -> atSetpoint());
        m_log_profile_done.log(() -> profileDone());
        m_log_at_goal.log(() -> atGoal());
    }

    ///////////////////////////////////////////
    ///////////////////////////////////////////
    ///////////////////////////////////////////

    private void actuateProfiledImpl(double unwrappedGoalX) {
        initReference(new StateR1(unwrappedGoalX, 0));
        SetpointsR1 unwrappedSetpoint = m_ref.get();
        m_nextUnwrappedSetpoint = unwrappedSetpoint.next();
        actuate(unwrappedSetpoint);
    }

    /** The reference only understands unwrapped angles. */
    private void initReference(StateR1 unwrappedGoal) {
        if (DEBUG) {
            System.out.printf("initReference old %s new %s\n", m_unwrappedGoal, unwrappedGoal);
        }
        if (unwrappedGoal.near(m_unwrappedGoal, m_positionTolerance, m_velocityTolerance) && m_ref.valid()) {
            // If the new goal is the same as the old goal, no change is needed.
            if (DEBUG)
                System.out.println("keep old goal");
            return;
        }
        // The new goal is not the same as the old goal, so tell the reference about it.
        m_unwrappedGoal = unwrappedGoal;
        if (DEBUG)
            System.out.println("replace goal");
        m_ref.setGoal(unwrappedGoal);
        // make sure the setpoint is near the measurement
        if (m_nextUnwrappedSetpoint == null) {
            // erased by dutycycle control, use measurement
            m_nextUnwrappedSetpoint = new ControlR1(m_mechanism.getUnwrappedPositionRad(), 0);
        }

        // initialize with the setpoint, not the measurement, to avoid noise.
        m_ref.init(m_nextUnwrappedSetpoint.state());
    }

}
