package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.util.Banner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final Machinery m_machinery;
    private final Binder m_binder;
    private final Autons m_autons;

    public Robot() {
        Banner.printBanner();
        enableLiveWindowInTest(false);
        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version);
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        System.out.printf("Identity: %s\n", Identity.instance.name());
        DriverStation.silenceJoystickConnectionWarning(true);
        Experiments.instance.show();
        SmartDashboard.putData(CommandScheduler.getInstance());
        CommandScheduler.getInstance().setPeriod(100);
        m_machinery = new Machinery();
        m_binder = new Binder(m_machinery);
        m_autons = new Autons(m_machinery);
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_machinery.periodic();
    }

    @Override
    public void close() {
        super.close();
        m_machinery.close();
        m_autons.close();
        m_binder.close();
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }
}
