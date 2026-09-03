package org.team100.lib.subsystems.tank;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.differential.DifferentialDriveDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.motor.rev.NeoCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.util.CanId;

public class TankDriveFactory {

    // a good value of dynamics might be SE2Dynamics(15, 0.5)
    // 15kg mass, 0.5 kg m^2 inertia?
    public static TankDrive make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId canL,
            CanId canR,
            double trackWidthM,
            double maxSpeedM_S,
            double gearRatio,
            double wheelDiaM,
            DifferentialDriveDynamics dynamics) {
        LoggerFactory log = parent.name("Tank Drive");
        LoggerFactory logL = log.name("left");
        LoggerFactory logR = log.name("right");

        Friction friction = new Friction(0.2, 0.2, 0.0, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(0.005);

        // ensure the simulated motor can go fast enough.
        double freeSpeedRad_S = maxSpeedM_S * gearRatio / (0.5 * wheelDiaM);

        Motor motorL = getMotor(
                logL, currentLog, freeSpeedRad_S, canL,
                MotorPhase.FORWARD, limit, friction, pid);
        Motor motorR = getMotor(
                logR, currentLog, freeSpeedRad_S, canR,
                MotorPhase.REVERSE, limit, friction, pid);

        LinearMechanism mechL = new LinearMechanism(
                logL, motorL, motorL.encoder(),
                gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        LinearMechanism mechR = new LinearMechanism(
                logR, motorR, motorR.encoder(),
                gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);

        return new TankDrive(
                log, fieldLogger, dynamics, trackWidthM, maxSpeedM_S, mechL, mechR);
    }

    private static Motor getMotor(
            LoggerFactory log,
            TotalCurrentLog currentLog,
            double freeSpeedRad_S,
            CanId can,
            MotorPhase phase,
            CurrentLimit limit,
            Friction friction,
            PIDConstants pid) {
        // parameters for velocity control.
        int averageDepth = 2;
        int measurementPeriod = 4;
        return switch (Identity.instance) {
            case BLANK -> new SimulatedMotor(log, freeSpeedRad_S);
            case DEMO_BOT -> new Neo550CANSparkMotor(
                    log, currentLog, can, NeutralMode100.BRAKE, phase,
                    limit, friction, pid, averageDepth, measurementPeriod);
            default -> new NeoCANSparkMotor(
                    log, currentLog, can, NeutralMode100.BRAKE, phase,
                    limit, friction, pid, averageDepth, measurementPeriod);
        };
    }
}
