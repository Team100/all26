package org.team100.lib.examples.motion;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Sometimes you don't need fancy positional profiles and feedback controls, you
 * just need something to spin on demand.
 * 
 * A common example is intake rollers: they don't need velocity control, they
 * just need to spin.
 * 
 * This example illustrates this use-case: the simplest possible open-loop
 * control.
 * 
 * This class extends SubsystemBase, to be compatible with the scheduler, and
 * the logger will use the class name.
 */
public class OpenLoopSubsystem extends SubsystemBase {
    private final BareMotor m_motor;

    public OpenLoopSubsystem(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory log = parent.type(this);
        /*
         * Here we use the Team 100 "Identity" mechanism to allow different
         * configurations for different hardware. The most important distinction here is
         * for simulation.
         */
        switch (Identity.instance) {
            case COMP_BOT -> {
                CanId canId = new CanId(1);
                CurrentLimit limit = new CurrentLimit(90, 60);
                PIDConstants pid = PIDConstants.makeVelocityPID(0.05);
                Friction friction = new Friction( 0.100, 0.100, 0.0, 0.1);
                m_motor = new Falcon500Motor(
                        log, currentLog, canId,
                        NeutralMode100.COAST, MotorPhase.FORWARD,
                        limit, friction, pid);
            }
            default -> {
                m_motor = new SimulatedBareMotor(log, 600);
            }
        }
    }

    ///////////////////////////////////////////////////////
    //
    // ACTIONS
    //
    // These methods make the subsystem do something.

    public void setDutyCycle(double dutyCycle) {
        m_motor.setDutyCycle(dutyCycle);
    }

    public void setVelocity(double velocity) {
        m_motor.setVelocity(velocity, 0);
    }

    ///////////////////////////////////////////////////////
    //
    // COMMANDS
    //
    // For single-subsystem actions, these actuator commands are the cleanest way to
    // do it. Multi-subsystem actions would need to use the methods above.
    //

    /** set duty cycle perpetually */
    public Command forward() {
        return run(() -> {
            setDutyCycle(1.0);
        });
    }

    public Command reverse() {
        return run(() -> {
            setDutyCycle(-1.0);
        });
    }

    @Override
    public void periodic() {
        m_motor.periodic();
    }
}
