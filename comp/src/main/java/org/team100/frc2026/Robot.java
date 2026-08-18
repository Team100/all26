package org.team100.frc2026;

import org.team100.frc2026.auton.AllAutons;
import org.team100.frc2026.robot.Binder;
import org.team100.frc2026.robot.Machinery;
import org.team100.frc2026.robot.Prewarmer;
import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.network.Sync;
import org.team100.lib.util.Banner;
import org.team100.lib.visualization.AutonVisualization;

import com.revrobotics.util.StatusLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.reduxrobotics.canand.CanandEventLoop;

/**
 * This is the main robot class, which wires up events from TimedRobot100.
 */
public class Robot extends TimedRobot100 {
    private final Sync sync;
    private final RobotLog m_robotLog;
    private final Machinery m_machinery;
    private final AutonVisualization m_autoViz;
    private final AllAutons m_allAutons;
    private final Binder m_binder;

    public Robot() {
        Banner.printBanner();

        CanandEventLoop.getInstance();

        // We want the CommandScheduler, not LiveWindow.
        enableLiveWindowInTest(false);

        // This is for setting up LaserCAN devices.
        // CanBridge.runTCP();
        StatusLogger.disableAutoLogging();
        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version);
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        System.out.printf("Identity: %s\n", Identity.instance.name());
        RobotController.setBrownoutVoltage(5.5);
        DriverStation.silenceJoystickConnectionWarning(true);
        Experiments.instance.show();

        // Log what the scheduler is doing. Use "withName()".
        SmartDashboard.putData(CommandScheduler.getInstance());
        // Set the period to forever, to make the watchdog shut up.
        CommandScheduler.getInstance().setPeriod(100);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        sync = new Sync(inst);

        m_robotLog = new RobotLog();

        m_machinery = new Machinery(m_robotLog.totalCurrentLog());
        m_binder = new Binder(m_machinery);

        m_allAutons = new AllAutons(m_machinery);

        LoggerFactory fieldLogger = Logging.instance().fieldLogger;
        m_autoViz = new AutonVisualization(fieldLogger);
        m_allAutons.onChange(m_autoViz::show);
        Prewarmer.init(m_machinery);
    }

    /** Called in the main loop. */
    @Override
    public void robotPeriodic() {
        // Advance the drumbeat.
        Takt.update();
        // reply to sync requests.
        sync.run();
        // Take all the measurements we can, as soon and quickly as possible.
        Cache.refresh();
        // Run one iteration of the command scheduler.
        CommandScheduler.getInstance().run();
        m_machinery.periodic();
        m_robotLog.periodic();
        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            // StrUtil.warn("FLUSHING EVERY LOOP, DO NOT USE IN COMP");
            NetworkTableInstance.getDefault().flush();
        }
    }

    //////////////////////////////////////////////////////////////////////
    //
    // INITIALIZERS, DO NOT CHANGE THESE
    //

    /** Forces the robot pose to the starting pose for the auton. */
    @Override
    public void autonomousInit() {
        AnnotatedCommand ac = m_allAutons.getAnnotated();
        if (ac == null)
            return;
        Pose2d start = ac.start();
        if (start != null) {
            m_machinery.resetPose(start);
        }
        Command auton = ac.command();
        if (auton == null)
            return;
        CommandScheduler.getInstance().schedule(auton);
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testInit() {
        System.out.println("*************************************");
        System.out.println("TEST MODE!");
        System.out.println("To run tests, hold down 'a' and 'b'");
    }

    @Override
    public void close() {
        super.close();
        m_machinery.close();
        m_allAutons.close();
        m_binder.close();
    }

    //////////////////////////////////////////////////////////////////////
    //
    // EXIT: CLEAN UP
    //

    @Override
    public void disabledExit() {
        m_autoViz.clear();
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // LEAVE ALL THESE EMPTY
    //

    @Override
    public void robotInit() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testExit() {
    }

}
