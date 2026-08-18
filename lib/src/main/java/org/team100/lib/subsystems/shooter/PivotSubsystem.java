package org.team100.lib.subsystems.shooter;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An example of a shooter pivot.
 */
public class PivotSubsystem extends SubsystemBase {

    private final BareMotor m_pivot;
    private final DoubleLogger m_log_angle;

    public PivotSubsystem(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId canId) {
        LoggerFactory logger = parent.type(this);
        m_log_angle = logger.doubleLogger(Level.TRACE, "Angle (rad)");
        m_pivot = (switch (Identity.instance) {
            case BLANK ->
                new SimulatedBareMotor(logger, 600);
            default -> new Neo550CANSparkMotor(
                    logger,
                    currentLog,
                    canId,
                    NeutralMode100.BRAKE,
                    MotorPhase.FORWARD, limit,
                    new Friction(0.07, 0.07, 0.01, 0.5),
                    PIDConstants.zero(),
                    0,
                    0);
        });
    }

    public void dutyCycle(double set) {
        m_pivot.setDutyCycle(set);
    }

    public double getAngleRad() {
        return m_pivot.getUnwrappedPositionRad();
    }

    public void setEncoderPosition(double positionRad) {
        m_pivot.setUnwrappedEncoderPositionRad(positionRad);
    }

    public void setTorqueLimit(double value) {
        m_pivot.setTorqueLimit(value);
    }

    public void zero() {
        m_pivot.stop();
    }

    public Command stop() {
        return run(this::zero);
    }

    @Override
    public void periodic() {
        m_pivot.periodic();
        m_log_angle.log(this::getAngleRad);
    }
}
