package org.team100.lib.subsystems.five_bar;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.kinematics.five_bar.FiveBarKinematics;
import org.team100.lib.kinematics.five_bar.JointPositions;
import org.team100.lib.kinematics.five_bar.Scenario;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.sensor.position.absolute.HomingRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Control at the "mechanism" level, which in this case means sending position
 * commands to the motor controller, without feedforward.
 */
public class FiveBarMech extends SubsystemBase {
    private static final boolean DEBUG = false;
    private static final boolean ENFORCE_FEASIBILITY = false;
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 10;
    private static final double STATOR_LIMIT = 10;
    private static final double Q1_MIN = 0;
    private static final double Q1_MAX = 2 * Math.PI / 3 - 0.1;
    private static final double Q5_MIN = Math.PI / 3 + 0.1;
    private static final double Q5_MAX = Math.PI;

    private final Scenario m_scenario;
    private final FiveBarKinematics m_kinematics;

    /** Left motor, "P1" in the diagram. */
    private final RotaryMechanism m_mechP1;
    /** Right motor, "P5" in the diagram. */
    private final RotaryMechanism m_mechP5;

    private final BareMotor m_motorP1;
    private final BareMotor m_motorP5;

    /**
     * There's no absolute encoder in the apparatus, so we use a homing sensor.
     */
    private final HomingRotaryPositionSensor m_sensorP1;
    private final HomingRotaryPositionSensor m_sensorP5;

    public FiveBarMech(LoggerFactory parent, TotalCurrentLog currentLog, Scenario scenario) {
        LoggerFactory logger = parent.type(this);
        LoggerFactory loggerP1 = logger.name("p1");
        LoggerFactory loggerP5 = logger.name("p5");
        m_scenario = scenario;

        m_kinematics = new FiveBarKinematics(logger);

        switch (Identity.instance) {
            case SWERVE_TWO -> {
                Falcon500Motor motorP1 = makeMotor(loggerP1, currentLog, new CanId(1));
                Falcon500Motor motorP5 = makeMotor(loggerP5, currentLog, new CanId(5));
                m_motorP1 = motorP1;
                m_motorP5 = motorP5;

                m_sensorP1 = new HomingRotaryPositionSensor(
                        new ProxyRotaryPositionSensor(motorP1.encoder(), 1.0));
                m_sensorP5 = new HomingRotaryPositionSensor(
                        new ProxyRotaryPositionSensor(motorP5.encoder(), 1.0));

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
            }
            default -> {
                SimulatedBareMotor motorP1 = new SimulatedBareMotor(loggerP1, 600);
                SimulatedBareMotor motorP5 = new SimulatedBareMotor(loggerP5, 600);
                m_motorP1 = motorP1;
                m_motorP5 = motorP5;

                m_sensorP1 = new HomingRotaryPositionSensor(
                        new ProxyRotaryPositionSensor(
                                motorP1.encoder(), 1.0));
                m_sensorP5 = new HomingRotaryPositionSensor(
                        new ProxyRotaryPositionSensor(
                                motorP5.encoder(), 1.0));

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
            }
        }
    }

    /** Update position by adding. */
    public void add(double p1, double p5) {
        double q1 = m_mechP1.getUnwrappedPositionRad() + p1;
        double q5 = m_mechP5.getUnwrappedPositionRad() + p5;
        setPosition(q1, q5);
    }

    /** Set position goal, motionless. */
    public void setPosition(double p1, double p5) {
        if (DEBUG)
            System.out.printf("FiveBarMech.setPosition %f %f\n", p1, p5);
        if (ENFORCE_FEASIBILITY) {
            if (!feasible(p1, p5)) {
                if (DEBUG)
                    System.out.println("infeasible!");
                return;
            }
        }
        m_mechP1.setUnwrappedPosition(p1, 0, 0);
        m_mechP5.setUnwrappedPosition(p5, 0, 0);
    }

    public Optional<JointPositions> getJointPositions() {
        double q1 = m_mechP1.getWrappedPositionRad();
        double q5 = m_mechP5.getWrappedPositionRad();
        if (DEBUG)
            System.out.printf("joint positions %f %f\n", q1, q5);
        return m_kinematics.forward(m_scenario, q1, q5);
    }

    @Override
    public void periodic() {
        m_mechP1.periodic();
        m_mechP5.periodic();
    }

    /**
     * True if the specified angles result in a feasible configuration. For example,
     * if the arms are far apart, the middle links won't reach. If the arms are
     * pointing towards each other, that's also not allowed. Each arm also has
     * physical limits enforced here.
     */
    boolean feasible(double q1, double q5) {
        Optional<JointPositions> optQ = m_kinematics.forward(m_scenario, q1, q5);
        if (optQ.isEmpty()) {
            // too wide
            return false;
        }
        JointPositions q = optQ.get();
        if (q.P2().x() < q.P4().x()) {
            // inverted
            return false;
        }
        if (q1 < Q1_MIN)
            return false;
        if (q1 > Q1_MAX)
            return false;
        if (q5 < Q5_MIN)
            return false;
        if (q5 > Q5_MAX)
            return false;
        return true;
    }

    //////////////////////

    private Falcon500Motor makeMotor(LoggerFactory logger, TotalCurrentLog currentLog, CanId canId) {
        /** Units of positional PID are volts per revolution. */
        PIDConstants pid = PIDConstants.makePositionPID(2.0);
        Friction friction = new Friction(0, 0, 0, 0);
        return new Falcon500Motor(
                logger,
                currentLog,
                canId,
                NeutralMode100.COAST,
                MotorPhase.REVERSE,
                new CurrentLimit(STATOR_LIMIT, SUPPLY_LIMIT),
                friction,
                pid);
    }

    /** For homing; ignores feasibility and limits. */
    private void setDutyCycle(double p1, double p5) {
        m_mechP1.setDutyCycleUnlimited(p1);
        m_mechP5.setDutyCycleUnlimited(p5);
    }

    /**
     * The "home" position is the max value; the idea is that you've run the duty
     * cycle (gently) to the end of travel before pushing the "home" button.
     */
    private void setHomePosition() {
        m_motorP1.setUnwrappedEncoderPositionRad(Q1_MAX);
        m_motorP5.setUnwrappedEncoderPositionRad(Q5_MIN);
    }

    ///////////////////////
    //
    // Commands

    /** Move in the direction of home: q1 to max, q5 to min */
    public Command home() {
        return run(() -> setDutyCycle(0.01, -0.01));
    }

    /** Set the position sensor to the home position. */
    public Command zero() {
        return runOnce(this::setHomePosition);
    }

    /** Update position by adding. */
    public Command position(DoubleSupplier p1, DoubleSupplier p5) {
        return run(() -> add(p1.getAsDouble(), p5.getAsDouble()));
    }

}
