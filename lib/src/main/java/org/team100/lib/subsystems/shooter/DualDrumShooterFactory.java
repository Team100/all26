package org.team100.lib.subsystems.shooter;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.rev.MinionSparkMotor;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.profile.r1.AccelLimitedVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

/** Configuration of motors on the demobot shooter. */
public class DualDrumShooterFactory {

    public enum ShooterType {
        /** open-loop duty cycle. simple. */
        DUTY_CYCLE,
        /** closed-loop velocity, REV controller needs tuning to make this work */
        VELOCITY
    }

    private final LoggerFactory log;
    private final TotalCurrentLog currentLog;
    private final double fullDutyCycle;
    private final double fullSpeedM_S;
    private final CurrentLimit limit;
    private final CanId canL;
    private final CanId canR;
    private final double gearRatio;
    private final double wheelDiaM;
    private final boolean profiled;
    private final PDynamics m_dynamics;

    public DualDrumShooterFactory(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            double fullDutyCycle,
            double fullSpeedM_S,
            CurrentLimit limit,
            CanId canL,
            CanId canR,
            double gearRatio,
            double wheelDiaM,
            boolean profiled,
            PDynamics dynamics) {
        this.log = parent.name("Shooter");
        this.currentLog = currentLog;
        this.fullDutyCycle = fullDutyCycle;
        this.fullSpeedM_S = fullSpeedM_S;
        this.limit = limit;
        this.canL = canL;
        this.canR = canR;
        this.gearRatio = gearRatio;
        this.wheelDiaM = wheelDiaM;
        this.profiled = profiled;
        m_dynamics = dynamics;
    }

    public DualDrumShooter get(ShooterType type) {
        return switch (type) {
            case DUTY_CYCLE -> makeDutyCycleShooter();
            case VELOCITY -> makeVelocityShooter();
        };
    }

    public DualDrumDutyCycleShooter makeDutyCycleShooter() {
        LoggerFactory logL = log.name("left");
        LoggerFactory logR = log.name("right");
        Friction friction = new Friction(0.0, 0.0, 0.0, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(0.005);

        Motor left = getMotor(
                limit, logL, currentLog, 600, canL,
                MotorPhase.FORWARD, friction, pid);

        Motor right = getMotor(
                limit, logR, currentLog, 600, canR,
                MotorPhase.REVERSE, friction, pid);

        return new DualDrumDutyCycleShooter(
                log, fullDutyCycle, left, right);
    }

    public DualDrumVelocityShooter makeVelocityShooter() {
        LoggerFactory logL = log.name("left");
        LoggerFactory logR = log.name("right");

        Friction friction = new Friction(0.0, 0.0, 0.0, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(0.005);

        // for simulation
        double maxSpeedM_S = 10;
        double freeSpeedRad_S = maxSpeedM_S * gearRatio / (0.5 * wheelDiaM);

        LinearMechanism mechL = getMech(
                currentLog, limit, canL, gearRatio, wheelDiaM, logL, friction, pid,
                freeSpeedRad_S, MotorPhase.REVERSE);

        LinearMechanism mechR = getMech(
                currentLog, limit, canR, gearRatio, wheelDiaM, logR, friction, pid,
                freeSpeedRad_S, MotorPhase.FORWARD);

        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(10);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);

        return new DualDrumVelocityShooter(
                log,
                fullSpeedM_S,
                new OutboardLinearVelocityServo(logL, mechL, m_dynamics, ref, 1),
                new OutboardLinearVelocityServo(logR, mechR, m_dynamics, ref, 1),
                profiled);
    }

    private static LinearMechanism getMech(
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId canId,
            double gearRatio,
            double wheelDiaM,
            LoggerFactory log,
            Friction friction,
            PIDConstants pid,
            double freeSpeedRad_S,
            MotorPhase motorPhase) {
        Motor motor = getMotor(
                limit, log, currentLog, freeSpeedRad_S, canId,
                motorPhase, friction, pid);
        LinearMechanism mech = new LinearMechanism(
                log, motor, motor.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        return mech;
    }

    private static Motor getMotor(
            CurrentLimit limit,
            LoggerFactory log,
            TotalCurrentLog currentLog,
            double freeSpeedRad_S,
            CanId canId,
            MotorPhase phase,
            Friction friction,
            PIDConstants pid) {
        // parameters for velocity control.
        int averageDepth = 2;
        int measurementPeriod = 4;
        return switch (Identity.instance) {
            case BLANK ->
                new SimulatedMotor(log, freeSpeedRad_S);
            case DEMO_BOT -> new MinionSparkMotor(
                    log, currentLog, canId, NeutralMode100.BRAKE, phase,
                    limit, friction, pid, averageDepth, measurementPeriod);
            default -> new MinionSparkMotor(
                    log, currentLog, canId, NeutralMode100.BRAKE, phase,
                    limit, friction, pid, averageDepth, measurementPeriod);
        };
    }

}
