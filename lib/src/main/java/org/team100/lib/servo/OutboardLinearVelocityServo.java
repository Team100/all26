package org.team100.lib.servo;

import org.team100.lib.coherence.Takt;
import org.team100.lib.dynamics.p.PAcceleration;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.dynamics.p.PEffort;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.VelocityControlR1Logger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.state.VelocityControlR1;

import edu.wpi.first.math.MathUtil;

/**
 * Profiled or direct velocity control using the feedback controller in the
 * motor controller hardware.
 * 
 * WARNING: the velocity control in REV motors is not very good. If you must use
 * it, you'll want to reduce the filtering on the sensor.
 */
public class OutboardLinearVelocityServo implements LinearVelocityServo {
    private static final boolean DEBUG = false;

    private final LinearMechanism m_mechanism;
    private final PDynamics m_dynamics;
    private final VelocityReferenceR1 m_ref;
    private final double m_toleranceM_S;

    private final DoubleLogger m_log_goal;
    private final VelocityControlR1Logger m_log_control;
    private final DoubleLogger m_log_velocity;

    // For calculating acceleration
    private double m_prevGoal = 0;
    // For calculating acceleration
    private double m_prevT = 0;

    private Double m_goal;
    /** Setpoints m/s and m/s^2 */
    private VelocityControlR1 m_nextSetpoint;

    public OutboardLinearVelocityServo(
            LoggerFactory parent,
            LinearMechanism mechanism,
            PDynamics dynamics,
            VelocityReferenceR1 ref,
            double toleranceM_S) {
        LoggerFactory log = parent.type(this);
        m_mechanism = mechanism;
        m_dynamics = dynamics;
        m_ref = ref;
        m_toleranceM_S = toleranceM_S;
        m_log_goal = log.doubleLogger(Level.COMP, "goal (m_s)");
        m_log_control = log.VelocityControlR1Logger(Level.COMP, "control (m_s)");
        m_log_velocity = log.doubleLogger(Level.COMP, "velocity (m_s)");
    }

    /**
     * Make a servo from a motor and a velocity reference.
     * Creates the mechanism in between.
     */
    public static OutboardLinearVelocityServo make(
            LoggerFactory log,
            BareMotor motor,
            PDynamics dynamics,
            VelocityReferenceR1 ref,
            double gearRatio,
            double wheelDiameterM,
            double toleranceM_S) {
        LinearMechanism mech = new LinearMechanism(
                log, motor, motor.encoder(), gearRatio, wheelDiameterM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        return new OutboardLinearVelocityServo(
                log, mech, dynamics, ref, toleranceM_S);
    }

    @Override
    public void reset() {
        if (DEBUG)
            System.out.println("WARNING: make sure resetting encoder position doesn't break anything");
        // m_mechanism.resetEncoderPosition();
        VelocityControlR1 measurement = new VelocityControlR1(getVelocity(), 0);
        m_nextSetpoint = measurement;
        m_ref.setGoal(measurement.v());
        m_ref.init(measurement.v());
    }

    /**
     * Resets the profile if necessary.
     * 
     * Must be called periodically to progress through the profile.
     * 
     * Will not work in Command.initialize().
     */
    @Override
    public void setVelocityProfiled(double goalM_S) {
        m_log_goal.log(() -> goalM_S);
        if (m_goal == null || !MathUtil.isNear(goalM_S, m_goal, m_toleranceM_S)
                || !m_ref.valid()) {
            m_goal = goalM_S;
            m_ref.setGoal(goalM_S);
            if (m_nextSetpoint == null) {
                m_nextSetpoint = new VelocityControlR1(getVelocity(), 0);
            }
            m_ref.init(m_nextSetpoint.v());
        }
        actuate(m_ref.get());
    }

    /** Invalidates the current profile */
    @Override
    public void setDutyCycle(double dutyCycle) {
        m_goal = null;
        m_nextSetpoint = null;
        m_mechanism.setDutyCycle(dutyCycle);
    }

    /**
     * Passthrough to the outboard control.
     * Invalidates the current profile.
     */
    @Override
    public void setVelocityDirect(double setpointM_S) {
        setVelocityDirect(setpointM_S, accel(setpointM_S));
    }

    /**
     * Passthrough to the outboard control.
     * Invalidates the current profile.
     * Uses the same setpoint for "current" and "next".
     * TODO: expose both setpoints here.
     */
    @Override
    public void setVelocityDirect(double setpointM_S, double setpointM_S2) {
        m_goal = null;
        VelocityControlR1 setpoint = new VelocityControlR1(setpointM_S, setpointM_S2);
        actuate(setpoint);
    }

    private void actuate(VelocityControlR1 setpoints) {
        if (setpoints == null)
            throw new IllegalArgumentException();
        m_nextSetpoint = setpoints;
        double velocityM_S = m_nextSetpoint.v();
        double accelM_S2 = m_nextSetpoint.a();
        PEffort t = m_dynamics.effort(new PAcceleration(accelM_S2));
        m_mechanism.setVelocity(velocityM_S, t.f());
        m_log_control.log(() -> m_nextSetpoint);
    }

    /**
     * Current velocity measurement. Note this can be noisy, maybe filter it.
     */
    @Override
    public double getVelocity() {
        return m_mechanism.getVelocityM_S();
    }

    /**
     * Velocity error (m/s).
     * Positive error = too slow, negative error = too fast.
     */
    public double error() {
        if (m_goal == null) {
            // TODO: no setpoint should yield null
            return 0;
        }
        return m_goal - m_mechanism.getVelocityM_S();
    }

    @Override
    public boolean atGoal() {
        return atSetpoint() && profileDone();
    }

    /** invalidates the current goal and setpoint */
    @Override
    public void stop() {
        m_goal = null;
        m_nextSetpoint = null;
        m_mechanism.stop();
    }

    @Override
    public double getDistance() {
        return m_mechanism.getPositionM();
    }

    @Override
    public void play(double freq) {
        m_mechanism.play(freq);
    }

    @Override
    public boolean atSetpoint() {
        if (m_nextSetpoint == null)
            return false;
        double vErr = m_nextSetpoint.v() - m_mechanism.getVelocityM_S();
        return Math.abs(vErr) < m_toleranceM_S;
    }

    @Override
    public boolean profileDone() {
        if (m_goal == null) {
            // if there's no profile, it's always done.
            return true;
        }
        return m_ref.profileDone();
    }

    @Override
    public void close() {
        m_mechanism.close();
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
        m_log_velocity.log(() -> getVelocity());
    }

    ////////////////////////////////////////////////

    /**
     * Acceleration from trailing difference in velocity.
     * 
     * Note: in simulation, if you pull the setpoint directly from the simulated
     * joystick input, acceleration will be choppy: zero acceleration every other
     * cycle, because the simulated inputs seem to be polled at only 10 hz.
     * 
     * To avoid this problem, use the SwerveLimiter.
     */
    private double accel(double setpoint) {
        double t = Takt.get();
        // TODO: experiment with fixed DT here, to reduce noise.
        double dt = t - m_prevT;
        m_prevT = t;
        double accel = (setpoint - m_prevGoal) / dt;
        m_prevGoal = setpoint;
        return accel;
    }
}
