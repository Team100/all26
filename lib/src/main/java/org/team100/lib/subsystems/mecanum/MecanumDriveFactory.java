package org.team100.lib.subsystems.mecanum;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.kinematics.mecanum.MecanumKinematics100.Slip;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.rev.NeoCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.ReduxGyro;
import org.team100.lib.util.CanId;

public class MecanumDriveFactory {

    public static MecanumDrive100 make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId gyroId,
            CanId canFL,
            CanId canFR,
            CanId canRL,
            CanId canRR,
            double m,
            double I,
            double trackWidthM,
            double wheelbaseM,
            Slip slip,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Mecanum Drive");
        LoggerFactory logFL = log.name("frontLeft");
        LoggerFactory logFR = log.name("frontRight");
        LoggerFactory logRL = log.name("rearLeft");
        LoggerFactory logRR = log.name("rearRight");

        PIDConstants pid = PIDConstants.makeVelocityPID(0.01);
        Friction friction = new Friction(0.5, 0.5, 0.0, 0.5);

        Gyro gyro = gyro(log, gyroId);
        slip = slip(slip);

        Motor motorFL = getMotor(
                logFL, currentLog, canFL, MotorPhase.REVERSE,
                limit, friction, pid);
        Motor motorFR = getMotor(
                logFR, currentLog, canFR, MotorPhase.FORWARD,
                limit, friction, pid);
        Motor motorRL = getMotor(
                logRL, currentLog, canRL, MotorPhase.REVERSE,
                limit, friction, pid);
        Motor motorRR = getMotor(
                logRR, currentLog, canRR, MotorPhase.FORWARD,
                limit, friction, pid);

        LinearMechanism mechFL = new LinearMechanism(
                logFL, motorFL, motorFL.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        LinearMechanism mechFR = new LinearMechanism(
                logFR, motorFR, motorFR.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        LinearMechanism mechRL = new LinearMechanism(
                logRL, motorRL, motorRL.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        LinearMechanism mechRR = new LinearMechanism(
                logRR, motorRR, motorRR.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        return new MecanumDrive100(
                log, fieldLogger, m, I, gyro, trackWidthM, wheelbaseM, slip,
                mechFL, mechFR, mechRL, mechRR);
    }

    public static Motor getMotor(
            LoggerFactory log, TotalCurrentLog currentLog, CanId can, MotorPhase phase,
            CurrentLimit limit, Friction friction, PIDConstants pid) {
        // parameters for velocity control
        int averageDepth = 2;
        int measurementPeriod = 4;
        return switch (Identity.instance) {
            case BLANK -> new SimulatedMotor(log, 600);
            default -> new NeoCANSparkMotor(
                    log, currentLog, can, NeutralMode100.BRAKE, phase,
                    limit, friction, pid, averageDepth, measurementPeriod);
        };
    }

    static Gyro gyro(LoggerFactory log, CanId gyroId) {
        if (gyroId == null)
            return null;
        return switch (Identity.instance) {
            case BLANK -> null;
            default -> new ReduxGyro(log, gyroId);
        };
    }

    static Slip slip(Slip slip) {
        return switch (Identity.instance) {
            case BLANK -> new Slip(1, 1, 1);// sim does not slip;
            default -> slip;
        };
    }

}
