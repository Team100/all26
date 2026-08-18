package org.team100.frc2025.grip;

import java.util.List;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.sim.LazySimulatedBareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.music.Music;
import org.team100.lib.music.Player;
import org.team100.lib.sensor.distance.LaserCan100;
import org.team100.lib.util.CanId;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The manipulator from the Calgames robot in 2025
 */
public class Manipulator extends SubsystemBase implements Music {

    private final BooleanLogger coralLogger;

    private static final int NEAR = 50;
    private final BareMotor m_algaeMotor;
    private final LinearMechanism m_leftMech;
    private final LinearMechanism m_rightMech;
    private final LinearMechanism m_algaeMech;
    private final LaserCan100 m_rightLaser;
    @SuppressWarnings("unused")
    private final LaserCan100 m_frontLaser;
    private final LaserCan100 m_backLaser;
    private final LaserCan100 m_leftLaser;

    private final List<Player> m_players;

    public Manipulator(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory log = parent.type(this);

        LoggerFactory algaeMotorLog = log.name("algae");
        LoggerFactory leftMotorLog = log.name("left");
        LoggerFactory rightMotorLog = log.name("right");
        coralLogger = log.booleanLogger(Level.TRACE, "Coral Detection");
        switch (Identity.instance) {
            case COMP_BOT -> {
                // Set specific parameters for the competition robot
                KrakenX60Motor leftMotor = new KrakenX60Motor(
                        leftMotorLog, currentLog, new CanId(19),
                        NeutralMode100.COAST,
                        MotorPhase.FORWARD,
                        new CurrentLimit(40, 40),
                        new Friction(0.900, 0.900, 0.0, 0.5),
                        PIDConstants.zero());
                KrakenX60Motor rightMotor = new KrakenX60Motor(
                        rightMotorLog, currentLog, new CanId(20), NeutralMode100.COAST,
                        MotorPhase.REVERSE,
                        new CurrentLimit(40, 40),
                        new Friction(0.900, 0.900, 0.0, 0.5),
                        PIDConstants.zero());
                KrakenX60Motor algaeMotor = new KrakenX60Motor(
                        algaeMotorLog, currentLog, new CanId(21), NeutralMode100.COAST,
                        MotorPhase.FORWARD,
                        new CurrentLimit(120, 120),
                        new Friction(0.900, 0.900, 0.0, 0.5),
                        PIDConstants.zero());
                algaeMotor.setTorqueLimit(4);
                m_algaeMotor = algaeMotor;
                m_rightLaser = new LaserCan100(new CanId(17));
                m_frontLaser = new LaserCan100(new CanId(16));
                m_backLaser = new LaserCan100(new CanId(18));
                m_leftLaser = new LaserCan100(new CanId(15));
                m_leftMech = new LinearMechanism(leftMotorLog, leftMotor, leftMotor.encoder(),
                        16, 0.1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_rightMech = new LinearMechanism(rightMotorLog, rightMotor, rightMotor.encoder(),
                        16, 0.1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_algaeMech = new LinearMechanism(algaeMotorLog, algaeMotor, algaeMotor.encoder(),
                        16, 0.1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            }
            default -> {
                SimulatedBareMotor leftMotor = new SimulatedBareMotor(log, 600);
                SimulatedBareMotor rightMotor = new SimulatedBareMotor(log, 600);
                // simulated algae motor gets overloaded 2 sec after starting
                LazySimulatedBareMotor algaeMotor = new LazySimulatedBareMotor(
                        log, new SimulatedBareMotor(log, 600), 2);
                m_algaeMotor = algaeMotor;
                m_leftMech = new LinearMechanism(log, leftMotor, leftMotor.encoder(),
                        1, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_rightMech = new LinearMechanism(log, rightMotor, rightMotor.encoder(),
                        1, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_algaeMech = new LinearMechanism(log, algaeMotor, algaeMotor.encoder(),
                        1, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_rightLaser = new LaserCan100();
                m_frontLaser = new LaserCan100();
                m_backLaser = new LaserCan100();
                m_leftLaser = new LaserCan100();
            }
        }
        m_players = List.of(m_leftMech, m_rightMech, m_algaeMech);

    }

    @Override
    public Command play(double freq) {
        return run(() -> {
            m_leftMech.play(freq);
            m_rightMech.play(freq);
            m_algaeMech.play(freq);
        });
    }

    @Override
    public List<Player> players() {
        return m_players;
    }

    /** Intake and hold. */
    public void intakeCenter() {
        if (hasCoral()) {
            stopMotors();
        } else if (hasCoralSideways()) {
            m_algaeMech.setDutyCycle(-1);
            m_leftMech.setDutyCycle(0.3);
            m_rightMech.setDutyCycle(-0.3);
        } else {
            m_algaeMech.setDutyCycle(-1);
            m_leftMech.setDutyCycle(1);
            m_rightMech.setDutyCycle(0.5);
        }
    }

    public boolean hasCoral() {
        if (Identity.instance.equals(Identity.BLANK))
            return false;
        return coralIsClose(m_backLaser);
    }

    public void ejectCenter() {
        m_algaeMech.setDutyCycle(1);
        m_leftMech.setDutyCycle(-0.75);
        m_rightMech.setDutyCycle(-0.75);
    }

    public void ejectCenterBack() {
        m_algaeMech.setDutyCycle(-1);
        m_leftMech.setDutyCycle(.75);
        m_rightMech.setDutyCycle(.75);
    }

    public void intakeSideways() {
        if (hasCoralSideways()) {
            stopMotors();
            m_algaeMech.setDutyCycle(-1);
        } else {
            m_algaeMech.setDutyCycle(-1);
            if (coralIsClose(m_leftLaser)) {
                m_leftMech.setDutyCycle(0.5);
                m_rightMech.setDutyCycle(-0.5);
            } else {
                m_leftMech.setDutyCycle(-0.5);
                m_rightMech.setDutyCycle(0.5);
            }
        }
    }

    public boolean intakingCoral() {
        return Math.abs(m_leftMech.getVelocityM_S()) > 0;
    }

    public boolean intakingAlgae() {
        return Math.abs(m_algaeMech.getVelocityM_S()) > 0;
    }

    public boolean hasCoralSideways() {
        return coralIsClose(m_leftLaser) && coralIsClose(m_rightLaser);
    }

    public void stopMotors() {
        m_algaeMech.setDutyCycle(0);
        m_leftMech.setDutyCycle(0);
        m_rightMech.setDutyCycle(0);
    }

    /**
     * Current is high when the algae is in.
     * (...and also at startup so include a delay.)
     */
    public boolean hasAlgae() {
        return m_algaeMotor.getCurrent() > 50;
    }

    /////////////////////////////////////////////////
    //
    // COMMANDS

    /** This is not "hold position" it is "disable". */
    public Command stop() {
        return startRun(this::lowAlgaeTorque, this::stopMotors);
    }

    public Command algaeHold() {
        return startRun(this::lowAlgaeTorque, this::intakeAlgae);
    }

    public Command algaeIntake() {
        return startRun(this::highAlgaeTorque, this::intakeAlgae);
    }

    public Command algaeEject() {
        return startRun(this::highAlgaeTorque, this::ejectAlgae);
    }

    /** Intake and hold. */
    public Command centerIntake() {
        return startRun(this::highAlgaeTorque, this::intakeCenter);
    }

    public Command sidewaysIntake() {
        return startRun(this::highAlgaeTorque, this::intakeSideways);
    }

    public Command sidewaysHold() {
        return startRun(this::lowCoralTorque, this::intakeSideways);
    }

    public Command centerEject() {
        return run(this::ejectCenter);
    }

    public Command centerEjectBack() {
        return run(this::ejectCenterBack);
    }

    //////////////////////////////////////////////////

    /**
     * Set high current limits.
     * Previous grip used 90A stator current, with a motor with kT of 0.018 Nm/amp,
     * so 1.62 Nm.
     */
    private void highAlgaeTorque() {
        m_algaeMotor.setTorqueLimit(3);
    }

    /**
     * Set moderate current limits.
     * Previous grip used 35A stator current, with a motor with kT of 0.018 Nm/amp,
     * so 0.63 Nm.
     */
    private void lowAlgaeTorque() {
        m_algaeMotor.setTorqueLimit(2.5);
    }

    private void lowCoralTorque() {
        m_algaeMotor.setTorqueLimit(.6);
    }

    private void intakeAlgae() {
        m_algaeMech.setDutyCycle(1);
    }

    public void ejectAlgae() {
        m_algaeMech.setDutyCycle(-1);
    }

    @Override
    public void periodic() {
        m_rightMech.periodic();
        m_leftMech.periodic();
        m_algaeMech.periodic();
        m_algaeMotor.periodic();
        coralLogger.log(this::hasCoral);
    }
    ///////////////////////////////////////////////

    private static boolean coralIsClose(LaserCan100 sensor) {
        Measurement m = sensor.getMeasurement();
        if (m == null)
            return false;
        return m.distance_mm < NEAR;
    }
}
