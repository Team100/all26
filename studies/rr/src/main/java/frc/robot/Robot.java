package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.util.Startup;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final Machinery m_machinery;
    private final Binder m_binder;
    private final Autons m_autons;

    public Robot() {
        Startup.start();
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
        // Poll for logs after all the actuation is done
        LogPoller.log();
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
