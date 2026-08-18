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
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simplest possible control of the five-bar: duty cycle only, no position
 * measurement.
 */
public class FiveBarBare extends SubsystemBase {
    private static final boolean DEBUG = false;
    /** Control from [-1,1] maps to [-SCALE,SCALE] duty cycle */
    private static final double SCALE = 0.01;
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 5;
    private static final double STATOR_LIMIT = 5;

    private final Scenario m_scenario;
    private final FiveBarKinematics m_kinematics;
    /** Left motor, "P1" in the diagram. */
    private final BareMotor m_motorP1;
    /** Right motor, "P5" in the diagram. */
    private final BareMotor m_motorP5;
    private final RotaryPositionSensor m_sensorP1;
    private final RotaryPositionSensor m_sensorP5;

    public FiveBarBare(LoggerFactory parent, TotalCurrentLog currentLog, Scenario scenario) {
        LoggerFactory logger = parent.type(this);
        LoggerFactory loggerP1 = logger.name("p1");
        LoggerFactory loggerP5 = logger.name("p5");

        m_scenario = scenario;
        m_kinematics = new FiveBarKinematics(logger);

        switch (Identity.instance) {
            case SWERVE_TWO -> {
                m_motorP1 = makeMotor(loggerP1, currentLog, new CanId(1));
                m_motorP5 = makeMotor(loggerP5, currentLog, new CanId(5));
            }
            default -> {
                m_motorP1 = new SimulatedBareMotor(loggerP1, 600);
                m_motorP5 = new SimulatedBareMotor(loggerP5, 600);
            }
        }
        m_sensorP1 = new ProxyRotaryPositionSensor(m_motorP1.encoder(), 1.0, 0.0);
        m_sensorP5 = new ProxyRotaryPositionSensor(m_motorP5.encoder(), 1.0, 0.0);
    }

    public Optional<JointPositions> getJointPositions() {
        double q1 = m_sensorP1.getWrappedPositionRad();
        double q5 = m_sensorP5.getWrappedPositionRad();
        if (DEBUG)
            System.out.printf("joint positions %f %f\n", q1, q5);
        return m_kinematics.forward(m_scenario, q1, q5);
    }

    /////////////////////

    private BareMotor makeMotor(LoggerFactory logger, TotalCurrentLog currentLog, CanId canId) {
        Friction friction = new Friction(0, 0, 0, 0);
        PIDConstants pid = PIDConstants.makePositionPID(2.0);
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

    private void setDutyCycle(double p1, double p5) {
        m_motorP1.setDutyCycle(p1);
        m_motorP5.setDutyCycle(p5);
    }

    /////////////////////
    //
    // Commands

    public Command dutyCycle(DoubleSupplier p1, DoubleSupplier p5) {
        return run(() -> setDutyCycle(
                SCALE * p1.getAsDouble(),
                SCALE * p5.getAsDouble()));
    }
}
