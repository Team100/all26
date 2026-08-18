package org.team100.frc2025.CalgamesArm;

import static edu.wpi.first.wpilibj2.command.Commands.select;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.prr.PRREffort;
import org.team100.lib.geometry.prr.PRRAcceleration;
import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.geometry.prr.PRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.kinematics.prr.PRRKinematics;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.AccelerationSE2Logger;
import org.team100.lib.logging.LoggerFactory.ConfigLogger;
import org.team100.lib.logging.LoggerFactory.PRRAccelerationLogger;
import org.team100.lib.logging.LoggerFactory.PRREffortLogger;
import org.team100.lib.logging.LoggerFactory.PRRVelocityLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.logging.LoggerFactory.VelocitySE2Logger;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.music.Music;
import org.team100.lib.music.Player;
import org.team100.lib.sensor.position.absolute.CombinedRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.GearedRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.sensor.position.incremental.ctre.Talon6Encoder;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.prr.SubsystemPRR;
import org.team100.lib.subsystems.prr.commands.FollowJointProfiles;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CalgamesMech extends SubsystemBase implements Music, PositionSubsystemSE2, SubsystemPRR {
    private static final boolean DEBUG = false;
    private boolean DISABLED = false;
    ////////////////////////////////////////////////////////
    ///
    /// CANONICAL CONFIGS
    /// These are used with profiles.
    ///
    public static final PRRConfig HOME = new PRRConfig(0, 0, 0);
    private static final PRRConfig CORAL_GROUND_PICK = new PRRConfig(0, -1.83, -0.12);
    private static final PRRConfig CLIMB = new PRRConfig(0, -1.83, 2);
    private static final PRRConfig STATION = new PRRConfig(0, -1, 0);
    private static final PRRConfig PROCESSOR = new PRRConfig(0, 1.2, 0);
    private static final PRRConfig ALGAE_GROUND = new PRRConfig(0, 1.43, 0);
    private static final PRRConfig L1 = new PRRConfig(0, -.95, -.5);

    ////////////////////////////////////////////////////////
    ///
    /// CANONICAL POSES
    /// These are used with trajectories.
    ///
    public static final Pose2d L2 = new Pose2d(0.56, 0.54, rad(2.0));
    public static final Pose2d L3 = new Pose2d(0.94, 0.56, rad(1.7));
    public static final Pose2d L4 = new Pose2d(1.57, 0.54, rad(2.0));
    private static final Pose2d L4_BACK = new Pose2d(1.92, -.54, rad(.75));
    public static final Pose2d ALGAE_L2 = new Pose2d(0.85, 0.7, rad(1.5));
    public static final Pose2d ALGAE_L3 = new Pose2d(1.15, 0.7, rad(1.5));
    public static final Pose2d BARGE = new Pose2d(2.3, -0.5, rad(-1.5));

    private final double m_armLengthM;
    private final double m_wristLengthM;
    private final MechTrajectories m_transit;

    private final PRRKinematics m_kinematics;

    private final Dynamics m_dynamics;

    private final ConfigLogger m_log_config;
    private final PRRVelocityLogger m_log_jointV;
    private final PRRAccelerationLogger m_log_jointA;
    private final PRREffortLogger m_log_jointF;

    private final Pose2dLogger m_log_pose;
    private final VelocitySE2Logger m_log_cartesianV;
    private final AccelerationSE2Logger m_log_cartesianA;

    private final LinearMechanism m_elevatorFront;
    private final LinearMechanism m_elevatorBack;

    private final RotaryMechanism m_shoulder;
    private final RotaryMechanism m_wrist;

    /** Home pose is Config(0,0,0), from forward kinematics. */
    private final Pose2d m_home;

    private final LoggerFactory m_profileLog;

    private final List<Player> m_players;

    public CalgamesMech(
            LoggerFactory log,
            TotalCurrentLog currentLog,
            double armLength,
            double wristLength) {
        LoggerFactory parent = log.type(this);
        m_profileLog = parent.name("profiles");
        m_armLengthM = armLength;
        m_wristLengthM = wristLength;

        m_kinematics = new PRRKinematics(armLength, wristLength, PRRKinematics.Solver.ANALYTIC);
        m_dynamics = new Dynamics();

        m_home = m_kinematics.forward(HOME);

        m_transit = new MechTrajectories(parent, this, m_kinematics);

        LoggerFactory jointLog = parent.name("joints");
        m_log_config = jointLog.logConfig(Level.DEBUG, "config");
        m_log_jointV = jointLog.logPRRVelocity(Level.DEBUG, "velocity");
        m_log_jointA = jointLog.logPRRAcceleration(Level.DEBUG, "accel");
        m_log_jointF = jointLog.logPRREffort(Level.DEBUG, "force");

        LoggerFactory cartesianLog = parent.name("cartesian");
        m_log_pose = cartesianLog.pose2dLogger(Level.DEBUG, "pose");
        m_log_cartesianV = cartesianLog.VelocitySE2Logger(Level.DEBUG, "velocity");
        m_log_cartesianA = cartesianLog.AccelerationSE2Logger(Level.DEBUG, "accel");

        LoggerFactory elevatorbackLog = parent.name("elevatorBack");
        LoggerFactory elevatorfrontLog = parent.name("elevatorFront");
        LoggerFactory shoulderLog = parent.name("shoulder");
        LoggerFactory wristLog = parent.name("wrist");

        switch (Identity.instance) {
            case COMP_BOT -> {

                final double elevatorGearRatio = 2.182;
                final double elevatorDrivePulleyDiameterM = 0.03844;
                final double elevatorLowerLimit = 0;
                final double elevatorUpperLimit = 2.1;

                KrakenX60Motor elevatorFrontMotor = new KrakenX60Motor(
                        elevatorfrontLog,
                        currentLog,
                        new CanId(11),
                        NeutralMode100.BRAKE, MotorPhase.REVERSE,
                        new CurrentLimit(100, 100),
                        new Friction(0.100, 0.100, 0.005, 0.5),
                        PIDConstants.makePositionPID(1));
                IncrementalBareEncoder elevatorFrontEncoder = elevatorFrontMotor.encoder();

                m_elevatorFront = new LinearMechanism(
                        elevatorfrontLog, elevatorFrontMotor, elevatorFrontEncoder,
                        elevatorGearRatio, elevatorDrivePulleyDiameterM,
                        elevatorLowerLimit, elevatorUpperLimit);

                KrakenX60Motor elevatorBackMotor = new KrakenX60Motor(
                        elevatorbackLog,
                        currentLog,
                        new CanId(12),
                        NeutralMode100.BRAKE, MotorPhase.FORWARD,
                        new CurrentLimit(100, 100),
                        new Friction(0.100, 0.100, 0.005, 0.5),
                        PIDConstants.makePositionPID(1));
                Talon6Encoder elevatorBackEncoder = elevatorBackMotor.encoder();
                m_elevatorBack = new LinearMechanism(
                        elevatorbackLog, elevatorBackMotor, elevatorBackEncoder,
                        elevatorGearRatio, elevatorDrivePulleyDiameterM,
                        elevatorLowerLimit, elevatorUpperLimit);

                KrakenX60Motor shoulderMotor = new KrakenX60Motor(
                        shoulderLog,
                        currentLog,
                        new CanId(24),
                        NeutralMode100.BRAKE,
                        MotorPhase.REVERSE,
                        new CurrentLimit(100, 100),
                        new Friction(0.100, 0.100, 0.005, 0.5),
                        PIDConstants.makePositionPID(1));
                Talon6Encoder shoulderEncoder = shoulderMotor.encoder();
                // The shoulder has a 5048 on the intermediate shaft
                AS5048RotaryPositionSensor shoulderSensor = new AS5048RotaryPositionSensor(
                        shoulderLog,
                        new RoboRioChannel(4),
                        0.684, // <<< This is the input offset (in TURNS) to adjust when zeroing
                        EncoderDrive.INVERSE);
                GearedRotaryPositionSensor gearedSensor = new GearedRotaryPositionSensor(
                        shoulderSensor,
                        8);

                ProxyRotaryPositionSensor shoulderProxySensor = new ProxyRotaryPositionSensor(
                        shoulderEncoder,
                        78);
                CombinedRotaryPositionSensor shoulderCombined = new CombinedRotaryPositionSensor(
                        shoulderLog, gearedSensor, shoulderProxySensor);
                m_shoulder = new RotaryMechanism(
                        shoulderLog, shoulderMotor, shoulderCombined,
                        78,
                        -2,
                        2);

                KrakenX60Motor wristMotor = new KrakenX60Motor(
                        wristLog,
                        currentLog,
                        new CanId(22),
                        NeutralMode100.COAST, MotorPhase.FORWARD,
                        new CurrentLimit(40, 60),
                        new Friction(0.100, 0.100, 0.005, 0.5),
                        PIDConstants.makePositionPID(1));
                // the wrist has no angle sensor, so it needs to start in the "zero" position.
                Talon6Encoder wristEncoder = wristMotor.encoder();
                final double wristGearRatio = 55.710;
                double wristEncoderOffset = 2.06818; // 2+0.06818
                m_wrist = new RotaryMechanism(
                        wristLog, wristMotor,
                        wristEncoder, wristEncoderOffset, wristGearRatio,
                        -1.5, // min
                        2.1); // max
            }
            default -> {
                SimulatedBareMotor elevatorMotorFront = new SimulatedBareMotor(
                        elevatorfrontLog, 600);
                IncrementalBareEncoder elevatorEncoderFront = elevatorMotorFront.encoder();
                m_elevatorFront = new LinearMechanism(
                        elevatorfrontLog, elevatorMotorFront, elevatorEncoderFront,
                        2, 0.05, 0, 2.2);

                SimulatedBareMotor elevatorMotorBack = new SimulatedBareMotor(
                        elevatorbackLog, 600);
                IncrementalBareEncoder elevatorEncoderBack = elevatorMotorBack.encoder();
                m_elevatorBack = new LinearMechanism(
                        elevatorbackLog, elevatorMotorBack, elevatorEncoderBack,
                        2, 0.05, 0, 2.2);

                SimulatedBareMotor shoulderMotor = new SimulatedBareMotor(
                        shoulderLog, 600);
                IncrementalBareEncoder shoulderEncoder = shoulderMotor.encoder();
                RotaryPositionSensor shoulderSensor = new SimulatedRotaryPositionSensor(
                        shoulderLog, shoulderEncoder, 100);
                m_shoulder = new RotaryMechanism(
                        shoulderLog, shoulderMotor, shoulderSensor, 100, -3, 3);

                SimulatedBareMotor wristMotor = new SimulatedBareMotor(
                        wristLog, 600);
                IncrementalBareEncoder wristEncoder = wristMotor.encoder();
                RotaryPositionSensor wristSensor = new SimulatedRotaryPositionSensor(
                        wristLog, wristEncoder, 58);
                m_wrist = new RotaryMechanism(
                        wristLog, wristMotor, wristSensor, 58, -3, 3);
            }
        }
        m_players = List.of(m_elevatorBack, m_elevatorFront, m_shoulder, m_wrist);
    }

    @Override
    public Command play(double freq) {
        return run(() -> {
            m_elevatorBack.play(freq);
            m_elevatorFront.play(freq);
            m_shoulder.play(freq);
            m_wrist.play(freq);
        });
    }

    @Override
    public List<Player> players() {
        return m_players;
    }

    public double getArmLength() {
        return m_armLengthM;
    }

    public double getHandLength() {
        return m_wristLengthM;
    }

    public PRRConfig getConfig() {
        return new PRRConfig(
                m_elevatorBack.getPositionM(),
                m_shoulder.getWrappedPositionRad(),
                m_wrist.getWrappedPositionRad());
    }

    public PRRVelocity getJointVelocity() {
        return new PRRVelocity(
                m_elevatorBack.getVelocityM_S(),
                m_shoulder.getVelocityRad_S(),
                m_wrist.getVelocityRad_S());
    }

    @Override
    public ModelSE2 getState() {
        PRRConfig c = getConfig();
        PRRVelocity jv = getJointVelocity();
        Pose2d p = m_kinematics.forward(c);
        VelocitySE2 v = m_kinematics.forward(c, jv);
        return new ModelSE2(p, v);
    }

    // for testing only
    public void setVelocity(VelocitySE2 v) {
        Pose2d pose = getState().pose();
        AccelerationSE2 a = new AccelerationSE2(0, 0, 0);
        ControlSE2 control = new ControlSE2(pose, v, a);

        PRRVelocity jv = m_kinematics.inverse(control.model());
        PRRAcceleration ja = m_kinematics.inverse(control);
        PRREffort jf = m_dynamics.forward(getConfig(), jv, ja);

        m_elevatorFront.setVelocity(jv.q1dot(), jf.f1());
        m_elevatorBack.setVelocity(jv.q1dot(), jf.f1());
        if (DISABLED) {
            m_wrist.setUnwrappedPosition(2, 0, 0);
            return;
        }
        m_wrist.setVelocity(jv.q3dot(), jf.t3());
        m_shoulder.setVelocity(jv.q2dot(), jf.t2());
    }

    /** There are no profiles here, so this control needs to be feasible. */
    @Override
    public void set(ControlSE2 control) {
        Pose2d pose = control.pose();
        PRRConfig config = m_kinematics.inverse(pose);
        if (DEBUG) {
            System.out.printf("pose %s config %s\n", StrUtil.pose2Str(pose), config);
        }
        if (config.isNaN()) {
            if (DEBUG)
                System.out.println("skipping invalid config");
            stop();
            return;
        }
        PRRVelocity jv = m_kinematics.inverse(control.model());
        PRRAcceleration ja = m_kinematics.inverse(control);
        set(config, jv, ja);
    }

    public void set(PRRConfig c, PRRVelocity jv, PRRAcceleration ja) {
        PRREffort jf = m_dynamics.forward(c, jv, ja);
        set(c, jv, ja, jf);
    }

    /** This is not "hold position" this is "torque off". */
    @Override
    public void stop() {
        m_elevatorFront.stop();
        m_elevatorBack.stop();
        m_shoulder.stop();
        m_wrist.stop();
    }

    /////////////////////////////////////////////////////////
    ///
    /// PROFILE COMMANDS
    ///

    /**
     * Use a profile to move from the current position and velocity to the "home"
     * position (origin) at rest, and end when done.
     */
    public Command profileHomeTerminal() {
        FollowJointProfiles f = MechProfiles.slowFast(m_profileLog,
                this, HOME);
        return f.until(f::isDone)
                .withName("profileHomeTerminal");
    }

    public Command profileHomeToL1() {
        FollowJointProfiles f = MechProfiles.fastSlow(m_profileLog,
                this, L1);
        return f.until(f::isDone)
                .withName("profileHomeTerminal");
    }

    /**
     * Use a profile to move from the current position and velocity to the "home"
     * position (origin) at rest, and hold there forever.
     */
    public Command profileHomeEndless() {
        return MechProfiles.slowFast(m_profileLog,
                this, HOME)
                .withName("profileHomeEndless");
    }

    /**
     * Use a profile to move from the current position and velocity to the "home"
     * position at rest, and then turn off the elevator motors so they stop trying
     * to push against gravity (making that squealing noise).
     */
    public Command profileHomeAndThenRest() {
        MoveAndHold f = MechProfiles.slowFast(m_profileLog, this, HOME);
        return sequence(
                // f.until(f::isDone),
                f.withTimeout(2),
                restAtHome() //
        ).withName("profileHomeAndThenRest");

    }

    /** Turn off the elevator motor. */
    public Command restAtHome() {
        return run(this::rest);
    }

    /**
     * Use a profile to move from the current position and velocity to the floor at
     * rest, and stay there forever.
     */
    public Command pickWithProfile() {
        return MechProfiles.fastSlow(m_profileLog,
                this, CORAL_GROUND_PICK)
                .withName("pickWithProfile");
    }

    /**
     * Use a profile to move from the current position and velocity to the floor at
     * rest, and stay there forever.
     */
    public Command algaePickGround() {
        return MechProfiles.algae(m_profileLog,
                this, ALGAE_GROUND)
                .withName("pickWithProfile");
    }

    public FollowJointProfiles homeGentle() {
        return MechProfiles.gentle(m_profileLog,
                this, HOME);
    }

    public FollowJointProfiles homeAlgae() {
        return MechProfiles.algaeUp(m_profileLog,
                this, HOME);
    }

    public Command climbWithProfile() {
        return MechProfiles.gentle(m_profileLog,
                this, CLIMB)
                .withName("climbWithProfile");
    }

    /**
     * Use a profile to move from the current position and velocity to the
     * station-pick location at rest, and stay there forever.
     */
    public Command stationWithProfile() {
        return MechProfiles.fastSlow(m_profileLog,
                this, STATION)
                .withName("stationWithProfile");
    }

    /**
     * Use a profile to move from the current position and velocity to the
     * processor location at rest, and stay there forever.
     */
    public Command processorWithProfile() {
        return MechProfiles.algae(m_profileLog,
                this, PROCESSOR)
                .withName("processorWithProfile");
    }

    //////////////////////////////////////////////////////////////////
    ///
    /// TRAJECTORY COMMANDS
    ///

    public MoveAndHold homeToL1() {
        return m_transit.endless("homeToL1",
                DirectionSE2.irrotational(-1.5),
                WaypointSE2.irrotational(L2, -1.7, 1.2));
    }

    // NEVER CALL
    public Command l1ToHome() {
        return m_transit.terminal("l1ToHome",
                DirectionSE2.irrotational(1.3),
                WaypointSE2.irrotational(m_home, 1.5, 1.2));
    }

    public MoveAndHold homeToL2() {
        return m_transit.endless("homeToL2",
                DirectionSE2.irrotational(1.5),
                WaypointSE2.irrotational(L2, 1.5, 1.2));
    }

    public Command l2ToHome() {
        return m_transit.terminal("l2ToHome",
                DirectionSE2.irrotational(-1.5),
                WaypointSE2.irrotational(m_home, -1.5, 1.2));
    }

    public MoveAndHold homeToL3() {
        return m_transit.endless("homeToL3",
                DirectionSE2.irrotational(0.8),
                WaypointSE2.irrotational(L3, 1.5, 1.2));
    }

    public Command l3ToHome() {
        return m_transit.terminal("l3ToHome",
                DirectionSE2.irrotational(-1.5),
                WaypointSE2.irrotational(m_home, -2.3, 1.2));
    }

    public MoveAndHold homeToL4() {
        return m_transit.endless("homeToL4",
                DirectionSE2.irrotational(0.1),
                WaypointSE2.irrotational(L4, 1.5, 1.2));
    }

    public MoveAndHold homeToL4Back() {
        return m_transit.endless("homeToL4",
                DirectionSE2.irrotational(0.1),
                WaypointSE2.irrotational(L4_BACK, -1.5, 1.2));
    }

    public Command l4ToHome() {
        return m_transit.terminal("l4ToHome",
                DirectionSE2.irrotational(-1.5),
                WaypointSE2.irrotational(m_home, -3, 1.2));
    }

    public Command l4BackToHome() {
        return m_transit.terminal("l4ToHome",
                DirectionSE2.irrotational(1.5),
                WaypointSE2.irrotational(m_home, -3, 1.2));
    }

    public Command homeToAlgaeL2() {
        return m_transit.endless("homeToAlgaeL2",
                DirectionSE2.irrotational(1.5),
                WaypointSE2.irrotational(ALGAE_L2, 1.5, 1.2));
    }

    public Command homeToAlgaeL3() {
        return m_transit.endless("homeToAlgaeL3",
                DirectionSE2.irrotational(0),
                WaypointSE2.irrotational(ALGAE_L3, 1.5, 1.2));
    }

    public Command algaeL2ToHome() {
        return m_transit.terminal("homeToAlgaeL2",
                DirectionSE2.irrotational(-1.0),
                WaypointSE2.irrotational(m_home, Math.PI, 1.2));
    }

    public Command algaeL3ToHome() {
        return m_transit.terminal("homeToAlgaeL3",
                DirectionSE2.irrotational(-1.0),
                WaypointSE2.irrotational(m_home, Math.PI, 1.2));
    }

    /**
     * Move to the supplied point for algae pick from the reef, and hold there
     * forever.
     */
    public Command algaeReefPick(Supplier<ScoringLevel> level) {
        return select(Map.ofEntries(
                Map.entry(ScoringLevel.L3, homeToAlgaeL3()),
                Map.entry(ScoringLevel.L2, homeToAlgaeL2()) //
        ), level)
                .withName("algaeReefPick");
    }

    public Command algaeReefExit(Supplier<ScoringLevel> level) {
        return select(Map.ofEntries(
                Map.entry(ScoringLevel.L3, algaeL3ToHome()),
                Map.entry(ScoringLevel.L2, algaeL2ToHome()) //
        ), level)
                .withName("algaeReefExit");
    }

    /**
     * Move to the barge scoring position and hold there forever
     */
    public MoveAndHold homeToBarge() {
        return m_transit.endless("homeToBarge",
                DirectionSE2.irrotational(0),
                WaypointSE2.irrotational(BARGE, -1, 1.2));
    }

    public MoveAndHold bargeToHome() {
        return m_transit.endless("bargeToHome",
                DirectionSE2.irrotational(2.5),
                WaypointSE2.irrotational(m_home, Math.PI, 1.2));

    }

    /** Not too far extended in any direction. */
    public boolean isSafeToDrive() {
        double x = m_elevatorBack.getPositionM();
        double y = m_shoulder.getWrappedPositionRad();
        return x < 1 && Math.abs(y) < 1;
    }

    /////////////////////////////////////////////////////////////

    public PRRKinematics getKinematics() {
        return m_kinematics;
    }

    @Override
    public void periodic() {
        m_shoulder.periodic();
        m_elevatorFront.periodic();
        m_elevatorBack.periodic();
        m_wrist.periodic();
    }

    /////////////////////////////////////////////////////////////

    /** Elevator torque off, shoulder and wrist hold position at zero. */
    private void rest() {
        m_elevatorFront.stop();
        m_elevatorBack.stop();
        if (DISABLED) {
            m_wrist.setUnwrappedPosition(2, 0, 0);
            return;
        }
        m_wrist.setUnwrappedPosition(0, 0, 0);
        m_shoulder.setUnwrappedPosition(0, 0, 0);
    }

    private void set(PRRConfig c, PRRVelocity jv, PRRAcceleration ja, PRREffort jf) {
        logConfig(c, jv, ja, jf);
        m_elevatorFront.setPosition(c.q1(), jv.q1dot(), jf.f1());
        m_elevatorBack.setPosition(c.q1(), jv.q1dot(), jf.f1());
        if (DISABLED) {
            m_wrist.setUnwrappedPosition(2, 0, 0);
            return;
        }
        m_wrist.setUnwrappedPosition(c.q3(), jv.q3dot(), jf.t3());
        m_shoulder.setUnwrappedPosition(c.q2(), jv.q2dot(), jf.t2());
    }

    public Command setDisabled(boolean disabled) {
        return run(() -> {
            DISABLED = disabled;
        });
    }

    private void logConfig(PRRConfig c, PRRVelocity jv, PRRAcceleration ja, PRREffort jf) {
        m_log_config.log(() -> c);
        m_log_jointV.log(() -> jv);
        m_log_jointA.log(() -> ja);
        m_log_jointF.log(() -> jf);
        Pose2d p = m_kinematics.forward(c);
        VelocitySE2 v = m_kinematics.forward(c, jv);
        AccelerationSE2 a = m_kinematics.forward(c, jv, ja);
        m_log_pose.log(() -> p);
        m_log_cartesianV.log(() -> v);
        m_log_cartesianA.log(() -> a);
    }

    /** to make the constants above easier to read */
    private static Rotation2d rad(double r) {
        return Rotation2d.fromRadians(r);
    };
}