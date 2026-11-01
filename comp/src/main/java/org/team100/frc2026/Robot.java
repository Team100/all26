package org.team100.frc2026;

import org.team100.frc2026.auton.Autons;
import org.team100.frc2026.robot.Binder;
import org.team100.frc2026.robot.Machinery;
import org.team100.frc2026.robot.Prewarmer;
import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.motor.ctre.CTREStartup;
import org.team100.lib.network.Sync;
import org.team100.lib.util.Startup;
import org.team100.lib.visualization.AutonVisualization;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * This is the main robot class, which wires up events from TimedRobot100.
 */
public class Robot extends TimedRobot100 {
    private final RobotLog m_robotLog;
    private final Sync m_sync;
    private final Machinery m_machinery;
    private final AutonVisualization m_autoViz;
    private final Autons m_autons;
    private final Binder m_binder;
    private final Command m_play;

    public Robot() {
        Startup.start();
        CTREStartup.start();
        System.out.printf("Robot class: %s\n",
                this.getClass().getName());
        LoggerFactory log = Logging.root();
        LoggerFactory fieldLogger = Logging.field();
        m_robotLog = new RobotLog(log);
        m_sync = new Sync(NetworkTableInstance.getDefault());
        m_machinery = new Machinery(log, fieldLogger, m_robotLog.totalCurrentLog());
        m_binder = new Binder(log, m_machinery);
        m_autons = new Autons(log, m_machinery);
        m_autoViz = new AutonVisualization(fieldLogger);
        m_autons.onChange(m_autoViz::show);
        m_play = Commands.repeatingSequence(m_machinery.play());
        Prewarmer.init(m_machinery);
        System.out.printf("Total Logger Keys: %d\n", Logging.keyCount());
    }

    /** Called in the main loop. */
    @Override
    public void robotPeriodic() {
        // Advance the drumbeat.
        Takt.update();
        // reply to sync requests.
        m_sync.run();
        // Take all the measurements we can, as soon and quickly as possible.
        Cache.refresh();
        // Run one iteration of the command scheduler.
        CommandScheduler.getInstance().run();
        // Poll for logs after all the actuation is done
        LogPoller.log();
        if (Experiments.INSTANCE.enabled(Experiment.FlushOften)) {
            NetworkTableInstance.getDefault().flush();
        }
        // System.out.printf("leftX %f\n", m_binder.m_driver.leftX());
        // System.out.printf("RightX %f\n", m_binder.m_driver.rightX());
        // System.out.printf("RightY %f\n", m_binder.m_driver.rightY());
        // System.out.printf("leftY %f\n", m_binder.m_driver.leftY());
        // System.out.printf("Right Trigger %f\n",
        // m_binder.m_driver.RightTriggerAxis());
        // System.out.printf(" Left Trigger %f\n", m_binder.m_driver.LeftTriggerAxis());

    }

    ///////////////////////////////////////////////////////////
    //
    // INITIALIZERS, DO NOT CHANGE THESE
    //

    /** Forces the robot pose to the starting pose for the auton. */
    @Override
    public void autonomousInit() {
        AnnotatedCommand ac = m_autons.getAnnotated();
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
        // don't show the auton in teleop
        m_autoViz.clear();
    }

    // TODO: revive test mode for 2027
    // @Override
    // public void testInit() {
    // System.out.println("*************************************");
    // System.out.println("TEST MODE!");
    // System.out.println("To run tests, hold down 'a' and 'b'");
    // }

    @Override
    public void close() {
        super.close();
        m_machinery.close();
        m_autons.close();
        m_binder.close();
    }

    ///////////////////////////////////////////////////////////
    //
    // EXIT: CLEAN UP
    //

    @Override
    public void disabledExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    ////////////////////////////////////////////////////////////
    //
    // LEAVE ALL THESE EMPTY
    //

    @Override
    public void simulationInit() {
    }

    @Override
    public void disabledInit() {
        // show the auton again when disabling.
        m_autoViz.show(m_autons.getAnnotated());
        // play music
        CommandScheduler.getInstance().schedule(m_play);
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
    public void autonomousExit() {
    }

    @Override
    public void teleopExit() {
    }
}
