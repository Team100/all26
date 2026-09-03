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
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearPositionServo;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

public class IndexerFactory {
    private static final double VELOCITY = 10;
    private static final double ACCEL = 20;

    public enum IndexerType {
        /** PWM indexer (servo) is the original design, and a fallback */
        PWM,
        /** open-loop duty cycle. simple. */
        DUTY_CYCLE,
        /** closed-loop velocity, hard to control single shots */
        VELOCITY,
        /** closed-loop position, easier to advance one ball at a time */
        POSITION
    }

    private final LoggerFactory log;
    private final RoboRioChannel channel;
    private final TotalCurrentLog currentLog;
    private final double fullDutyCycle;
    private final double fullVelocityM_S;
    private final CurrentLimit limit;
    private final CanId canId;
    private final double gearRatio;
    private final double wheelDiaM;
    private final boolean profiled;
    private final PDynamics m_dynamics;

    public IndexerFactory(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            RoboRioChannel channel,
            double fullDutyCycle,
            double fullVelocityM_S,
            CurrentLimit limit,
            CanId canId,
            double gearRatio,
            double wheelDiaM,
            boolean profiled,
            PDynamics dynamics) {
        this.log = parent.name("Indexer");
        this.channel = channel;
        this.currentLog = currentLog;
        this.fullDutyCycle = fullDutyCycle;
        this.fullVelocityM_S = fullVelocityM_S;
        this.limit = limit;
        this.canId = canId;
        this.gearRatio = gearRatio;
        this.wheelDiaM = wheelDiaM;
        this.profiled = profiled;
        m_dynamics = dynamics;
    }

    public ShooterIndexer get(IndexerType type) {
        return switch (type) {
            case PWM -> makePWMIndexer();
            case DUTY_CYCLE -> makeDutyCycleIndexer();
            case VELOCITY -> makeVelocityIndexer();
            case POSITION -> makePositionIndexer();
        };
    }

    public PWMIndexer makePWMIndexer() {
        if (channel == null)
            throw new IllegalArgumentException();
        return new PWMIndexer(log, channel);
    }

    public DutyCycleIndexer makeDutyCycleIndexer() {
        if (canId == null)
            throw new IllegalArgumentException();
        if (fullDutyCycle == 0)
            throw new IllegalArgumentException();
        Friction friction = new Friction(0.02, 0.02, 0.0, 0.5);
        // duty cycle, no feedback.
        PIDConstants pid = PIDConstants.zero();
        // for simulation
        double maxSpeedM_S = 10;
        double freeSpeedRad_S = maxSpeedM_S * gearRatio / (0.5 * wheelDiaM);
        Motor motor = getMotor(
                limit, log, currentLog, freeSpeedRad_S, canId,
                MotorPhase.FORWARD, friction, pid);
        return new DutyCycleIndexer(log, fullDutyCycle, motor);
    }

    public PositionIndexer makePositionIndexer() {
        if (canId == null)
            throw new IllegalArgumentException();
        LinearMechanism mech = getPositionMech(
                log, currentLog, limit, canId, gearRatio, wheelDiaM);
        TrapezoidProfileR1 profile = new TrapezoidProfileR1(
                VELOCITY, ACCEL, 0.02);
        ProfileReferenceR1 ref = new ProfileReferenceR1(
                log, () -> profile, 0.02, 0.02);
        OutboardLinearPositionServo servo = new OutboardLinearPositionServo(
                log, mech, m_dynamics, ref, 0.02, 0.02);
        return new PositionIndexer(log, servo, profiled);
    }

    public VelocityIndexer makeVelocityIndexer() {
        if (canId == null)
            throw new IllegalArgumentException();
        if (fullVelocityM_S == 0)
            throw new IllegalArgumentException();
        LinearMechanism mech = getVelocityMech(
                log, currentLog, limit,
                canId, gearRatio, wheelDiaM);
        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(100);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);
        OutboardLinearVelocityServo servo = new OutboardLinearVelocityServo(
                log, mech, m_dynamics, ref, 1);
        return new VelocityIndexer(log, fullVelocityM_S, servo, profiled);
    }

    private static LinearMechanism getPositionMech(
            LoggerFactory log,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId canId,
            double gearRatio,
            double wheelDiaM) {
        Friction friction = new Friction(0.02, 0.02, 0.00, 0.5);
        PIDConstants pid = PIDConstants.makePositionPID(0.5);
        // for simulation
        double maxSpeedM_S = 10;
        double freeSpeedRad_S = maxSpeedM_S * gearRatio / (0.5 * wheelDiaM);
        Motor motor = getMotor(
                limit, log, currentLog, freeSpeedRad_S, canId,
                MotorPhase.REVERSE, friction, pid);
        LinearMechanism mech = new LinearMechanism(
                log, motor, motor.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        return mech;
    }

    private static LinearMechanism getVelocityMech(
            LoggerFactory log,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId canId,
            double gearRatio,
            double wheelDiaM) {
        Friction friction = new Friction(0.02, 0.02, 0.0, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(0.005);
        // for simulation
        double maxSpeedM_S = 10;
        double freeSpeedRad_S = maxSpeedM_S * gearRatio / (0.5 * wheelDiaM);
        Motor motor = getMotor(
                limit, log, currentLog, freeSpeedRad_S, canId,
                MotorPhase.REVERSE, friction, pid);
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
        // motor params for for velocity control
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
