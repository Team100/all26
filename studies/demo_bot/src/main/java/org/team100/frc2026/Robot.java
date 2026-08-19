package org.team100.frc2026;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.dynamics.differential.DifferentialDriveDynamics;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.indicator.SolidIndicator;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.shooter.DualDrumShooter;
import org.team100.lib.subsystems.shooter.DualDrumShooterFactory;
import org.team100.lib.subsystems.shooter.DualDrumShooterFactory.ShooterType;
import org.team100.lib.subsystems.shooter.IndexerFactory;
import org.team100.lib.subsystems.shooter.IndexerFactory.IndexerType;
import org.team100.lib.subsystems.shooter.PivotSubsystem;
import org.team100.lib.subsystems.shooter.ShooterIndexer;
import org.team100.lib.subsystems.tank.TankDrive;
import org.team100.lib.subsystems.tank.TankDriveFactory;
import org.team100.lib.subsystems.tank.commands.TankManual;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;

public class Robot extends TimedRobot100 {
    private static final ShooterType SHOOTER = ShooterType.VELOCITY;
    /**
     * Ball velocity is the most sensitive variable influencing safety.
     * 
     * The 50% cornea contusion limit is 1500 J/m^2. The ball area is
     * about 0.005 m^2, so the KE limit is around 7.5 J, or around 20 m/s.
     * 
     * For more detail, see the design doc:
     * 
     * https://docs.google.com/document/d/14Ebyd1gQzmyat9dlTWtNtymE9XSHsTXXHNyspYBy_Ys
     */
    private static final double MAXIMUM_BALL_VELOCITY_M_S = 20;
    private static final double MAXIMUM_SHOOTER_DUTY_CYCLE = 0.2;
    private static final double SHOOTER_GEAR_RATIO = 1.00;
    private static final double SHOOTER_WHEEL_DIA_M = 0.16;

    private static final IndexerType INDEXER = IndexerType.VELOCITY;
    private static final double MAXIMUM_INDEXER_DUTY_CYCLE = 1.0;
    private static final double MAX_INDEXER_VELOCITY_M_S = 5;
    private static final double INDEXER_WHEEL_DIAMETER = 0.1;
    private static final double INDEXER_GEAR_RATIO = 1.0;

    private static final double FIVE_TO_ONE = 5.2307692308;
    private static final double DRIVE_GEAR_RATIO = FIVE_TO_ONE * FIVE_TO_ONE;
    private static final double DRIVE_WHEEL_DIAM = 0.098425;
    // measured 4/30/26, used to be 0.4.
    private static final double TRACK_WIDTH = 0.5;
    // slower in order to preserve the tires on the blacktop
    private static final double MAX_SPEED_M_S = 1.0;
    private static final double MAX_OMEGA_RAD_S = 2.0;

    private final SolidIndicator m_led;
    private final RobotLog m_robotLog;
    private final TankDrive m_drive;
    private final Command m_auton;
    private final DualDrumShooter m_shooter;
    private final ShooterIndexer m_indexer;
    private final PivotSubsystem m_pivot;

    public Robot() {
        Banner.printBanner();
        Experiments.instance.show();
        SmartDashboard.putData(CommandScheduler.getInstance());

        m_robotLog = new RobotLog();
        TotalCurrentLog m_currentLog = m_robotLog.totalCurrentLog();

        Logging logging = Logging.instance();
        LoggerFactory fieldLogger = logging.fieldLogger;
        LoggerFactory logger = logging.rootLogger;

        DriverXboxControl xbox = new DriverXboxControl(0);

        m_led = new SolidIndicator(new RoboRioChannel(0), 512);
        m_led.state(this::ledColor);
        // TODO: use machine state instead of buttons
        m_led.event(xbox::leftTrigger, Color.WHITE);
        DifferentialDriveDynamics driveDynamics = new DifferentialDriveDynamics(10, 1, 0.4);
        m_drive = TankDriveFactory.make(
                fieldLogger,
                logger,
                m_currentLog,
                new CurrentLimit(15, 15),
                new CanId(3),
                new CanId(2),
                TRACK_WIDTH,
                MAX_SPEED_M_S,
                DRIVE_GEAR_RATIO,
                DRIVE_WHEEL_DIAM,
                driveDynamics);
        // changed back to all-right-side because it's easier to explain
        m_drive.setDefaultCommand(new TankManual(
                logger,
                () -> xbox.rightY() * -1.0,
                () -> xbox.rightX() * -1.0,
                MAX_SPEED_M_S,
                MAX_OMEGA_RAD_S,
                m_drive));

        PDynamics pdynamics = PDynamics.drum(0.5, SHOOTER_WHEEL_DIA_M / 2);
        DualDrumShooterFactory shooterFactory = new DualDrumShooterFactory(
                logger,
                m_currentLog,
                MAXIMUM_SHOOTER_DUTY_CYCLE,
                MAXIMUM_BALL_VELOCITY_M_S,
                new CurrentLimit(20, 20),
                new CanId(39),
                new CanId(8),
                SHOOTER_GEAR_RATIO,
                SHOOTER_WHEEL_DIA_M,
                false,
                pdynamics);
        m_shooter = shooterFactory.get(SHOOTER);
        m_shooter.setDefaultCommand(m_shooter.stop());

        PDynamics idynamics = PDynamics.drum(0.1, INDEXER_WHEEL_DIAMETER / 2);
        IndexerFactory indexerFactory = new IndexerFactory(
                logger,
                m_currentLog,
                new RoboRioChannel(0),
                MAXIMUM_INDEXER_DUTY_CYCLE,
                MAX_INDEXER_VELOCITY_M_S,
                new CurrentLimit(30, 30),
                new CanId(7),
                INDEXER_GEAR_RATIO,
                INDEXER_WHEEL_DIAMETER,
                true,
                idynamics);
        m_indexer = indexerFactory.get(INDEXER);
        m_indexer.setDefaultCommand(m_indexer.stop());

        m_pivot = new PivotSubsystem(
                logger,
                m_currentLog,
                new CurrentLimit(20, 20),
                new CanId(5));
        m_pivot.setDefaultCommand(m_pivot.stop());
        // m_pivot.setDefaultCommand(
        // new PivotDefault(
        // () -> xbox.leftY() * -1.0,
        // m_pivot));

        /////////////////////////////////////////////////////////////////////////////////////
        ///
        /// SHOOTER CONTROLS
        ///
        // changed shooter to "b" because it's easier to see
        new Trigger(xbox::b)
                .whileTrue(
                        Commands.parallel(
                                m_shooter.spinSlow(),
                                Commands.waitUntil(m_shooter::atGoal)
                                        .andThen(m_indexer.single()
                                                .andThen(m_indexer.stop().withTimeout(0.5)))
                                        .repeatedly())
                                .withName("Shooter slow"));
        // new Trigger(xbox::leftTrigger).and(xbox::rightTrigger)
        // .whileTrue(m_shooter.spinFast()
        // .withName("Shooter fast"));
        // new Trigger(xbox::leftBumper)
        // .onTrue(m_indexer.single()
        // .withName("Indexer single"));
        // new Trigger(xbox::rightBumper)
        // .whileTrue(m_indexer.continuous()
        // .withName("Indexer continuous"));

        m_auton = null;
    }

    private Color ledColor() {
        double timeSec = Takt.get();
        double modTime = timeSec % 1;
        if (modTime < 0.04)
            return Color.ORANGE;
        return Color.BLACK;
    }

    /**
     * robotPeriodic is called in the IterativeRobotBase.loopFunc, which is what the
     * TimedRobot runs in the main loop.
     * 
     * This is what should do all the work.
     */
    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_robotLog.periodic();
        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    public void autonomousInit() {
        if (m_auton == null)
            return;
        CommandScheduler.getInstance().schedule(m_auton);
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        m_pivot.setEncoderPosition(Math.PI / 2);
    }

}