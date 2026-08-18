package org.team100.lib.subsystems.five_bar;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.kinematics.five_bar.ActuatorAngles;
import org.team100.lib.kinematics.five_bar.FiveBarKinematics;
import org.team100.lib.kinematics.five_bar.JointPositions;
import org.team100.lib.kinematics.five_bar.Scenario;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.Translation2dLogger;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.subsystems.five_bar.commands.Move;
import org.team100.lib.util.CanId;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Cartesian control using inverse kinematics, and without profiling.
 */
public class FiveBarCartesian extends SubsystemBase {
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 20;
    private static final double STATOR_LIMIT = 20;

    LoggerFactory m_logger;
    private final Scenario m_scenario;
    private final FiveBarKinematics m_kinematics;

    /** Left motor, "P1" in the diagram. */
    /**
     * There's no absolute encoder in the apparatus, so we use a "proxy" instead;
     * this needs a "homing" mechanism of some kind.
     */
    private final ProxyRotaryPositionSensor m_sensorP1;
    private final RotaryMechanism m_mechP1;

    /** Right motor, "P5" in the diagram. */
    private final ProxyRotaryPositionSensor m_sensorP5;
    private final RotaryMechanism m_mechP5;

    private final Translation2dLogger m_log_desired_position;
    private final Translation2dLogger m_log_position;
    private final BooleanLogger m_log_feasible;

    public FiveBarCartesian(LoggerFactory parent, TotalCurrentLog currentLog, Scenario scenario) {
        m_logger = parent.type(this);
        LoggerFactory loggerP1 = m_logger.name("p1");
        LoggerFactory loggerP5 = m_logger.name("p5");
        m_scenario = scenario;

        m_kinematics = new FiveBarKinematics(m_logger);

        m_log_desired_position = m_logger.translation2dLogger(Level.COMP, "desired postion");
        m_log_position = m_logger.translation2dLogger(Level.COMP, "position");
        m_log_feasible = m_logger.booleanLogger(Level.COMP, "feasible");

        // zeros
        PIDConstants pid = PIDConstants.makePositionPID(2.0);
        Friction friction = new Friction(0, 0, 0, 0);

        BareMotor motorP1;
        BareMotor motorP5;
        switch (Identity.instance) {
            case SWERVE_TWO -> {
                motorP1 = new Falcon500Motor(
                        loggerP1,
                        currentLog,
                        new CanId(1),
                        NeutralMode100.COAST,
                        MotorPhase.REVERSE,
                        new CurrentLimit(STATOR_LIMIT, SUPPLY_LIMIT),
                        friction,
                        pid);
                motorP5 = new Falcon500Motor(
                        loggerP5,
                        currentLog,
                        new CanId(5),
                        NeutralMode100.COAST,
                        MotorPhase.REVERSE,
                        new CurrentLimit(STATOR_LIMIT, SUPPLY_LIMIT),
                        friction,
                        pid);
            }
            default -> {
                motorP1 = new SimulatedBareMotor(loggerP1, 600);
                motorP5 = new SimulatedBareMotor(loggerP5, 600);
            }
        }

        m_sensorP1 = new ProxyRotaryPositionSensor(motorP1.encoder(), 1.0);
        m_sensorP5 = new ProxyRotaryPositionSensor(motorP5.encoder(), 1.0);
        m_mechP1 = new RotaryMechanism(
                loggerP1,
                motorP1,
                m_sensorP1,
                1.0,
                -100.0,
                100.0);
        m_mechP5 = new RotaryMechanism(
                loggerP5,
                motorP5,
                m_sensorP5,
                1.0,
                -100.0,
                100.0);

        // TODO: what to do for initial position?
        m_mechP1.setUnwrappedPosition(0, 0, 0);
        m_mechP5.setUnwrappedPosition(0, 0, 0);
    }

    /**
     * Moves both axes so that P3 reaches the specified translation relative to the
     * work center. Movement is uncoordinated, so the caller should manage the
     * trajectory, if desired.
     */
    public void setPosition(Translation2d t) {
        m_log_desired_position.log(() -> t);
        double x3 = t.getX();
        double y3 = t.getY();
        Optional<ActuatorAngles> optP = m_kinematics.inverse(
                m_scenario, x3 + m_scenario.xcenter, y3 + m_scenario.ycenter);
        if (optP.isEmpty()) {
            // skip infeasible
            m_log_feasible.log(() -> false);
            return;
        }
        ActuatorAngles p = optP.get();
        m_log_feasible.log(() -> true);
        m_mechP1.setUnwrappedPosition(p.q1(), 0, 0);
        m_mechP5.setUnwrappedPosition(p.q5(), 0, 0);
    }

    public Optional<JointPositions> getJointPositions() {
        double q1 = m_mechP1.getWrappedPositionRad();
        double q5 = m_mechP5.getWrappedPositionRad();
        return m_kinematics.forward(m_scenario, q1, q5);
    }

    /** Position *relative to work center */
    public Optional<Translation2d> getPosition() {
        double q1 = m_mechP1.getWrappedPositionRad();
        double q5 = m_mechP5.getWrappedPositionRad();
        Optional<JointPositions> optJ = m_kinematics.forward(m_scenario, q1, q5);
        if (optJ.isEmpty())
            return Optional.empty();
        JointPositions j = optJ.get();
        double x3 = j.P3().x();
        double y3 = j.P3().y();
        return Optional.of(new Translation2d(
                x3 - m_scenario.xcenter,
                y3 - m_scenario.ycenter));
    }

    @Override
    public void periodic() {
        m_mechP1.periodic();
        m_mechP5.periodic();
        Optional<Translation2d> p = getPosition();
        if (p.isPresent())
            m_log_position.log(() -> p.get());
    }

    //////////////////////

    private void setDutyCycle(double p1, double p5) {
        m_mechP1.setDutyCycle(p1);
        m_mechP5.setDutyCycle(p5);
    }

    /**
     * Sets the encoders to the position yielded by the "zero" command, i.e. running
     * all the way to the hard stop.
     */
    private void resetEncoderPosition() {
        // these match the real apparatus, more or less.
        m_sensorP1.setEncoderPosition(-0.35);
        m_sensorP5.setEncoderPosition(1.22);
    }

    ///////////////////////
    //
    // Commands

    /** Run in the negative direction as far as possible. */
    public Command home() {
        final double homingDutyCycle = -0.05;
        return run(() -> setDutyCycle(homingDutyCycle, homingDutyCycle));
    }

    public Command zero() {
        return runOnce(this::resetEncoderPosition);
    }

    public Command position(Supplier<Translation2d> t) {
        return run(() -> setPosition(t.get()));
    }

    public Command move(Translation2d goal) {
        Move m = new Move(m_logger, this, goal, 0.5);
        return m.until(m::done);
    }
}
