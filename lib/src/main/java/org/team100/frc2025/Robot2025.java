package org.team100.frc2025;

import org.team100.frc2025.robot.AllAutons2025;
import org.team100.frc2025.robot.Binder2025;
import org.team100.frc2025.robot.Machinery2025;
import org.team100.frc2025.robot.Prewarmer2025;
import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.util.Banner;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot2025 extends TimedRobot100 {

    private final RobotLog m_robotLog;
    private final Machinery2025 m_machinery;
    private final AllAutons2025 m_allAutons;
    private final Binder2025 m_binder;

    public Robot2025() {
        Banner.printBanner();

        // We want the CommandScheduler, not LiveWindow.
        enableLiveWindowInTest(false);

        // This is for setting up LaserCAN devices.
        // CanBridge.runTCP();

        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version);
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        System.out.printf("Identity: %s\n", Identity.instance.name());
        RobotController.setBrownoutVoltage(5.5);
        DriverStation.silenceJoystickConnectionWarning(true);
        Experiments.instance.show();

        // Log what the scheduler is doing. Use "withName()".
        SmartDashboard.putData(CommandScheduler.getInstance());

        m_robotLog = new RobotLog();

        m_machinery = new Machinery2025(m_robotLog.totalCurrentLog());
        m_allAutons = new AllAutons2025(m_machinery);
        m_binder = new Binder2025(m_machinery);
        m_binder.bind();

        Prewarmer2025.init(m_machinery);
    }

    @Override
    public void robotPeriodic() {
        // Advance the drumbeat.
        Takt.update();
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

    @Override
    public void autonomousInit() {
        Command auton = m_allAutons.get();
        if (auton == null)
            return;
        CommandScheduler.getInstance().schedule(auton);
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void close() {
        super.close();
        m_machinery.close();
        m_allAutons.close();
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
    public void testInit() {
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

}