package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.util.Startup;
import org.team100.rrr.auton.Autons;
import org.team100.rrr.robot.Binder;
import org.team100.rrr.robot.Machinery;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.networktables.NetworkTableInstance;

public class Robot extends TimedRobot100 {
    private final RobotLog m_robotLog;

    private final Machinery m_machinery;
    private final Binder m_binder;
    private final Autons m_autons;

    /** Mirrors the comp robot code. */
    public Robot() {
        Startup.start();
        LoggerFactory rootLogger = Logging.instance().rootLogger;
        m_robotLog = new RobotLog(rootLogger);
        m_machinery = new Machinery(m_robotLog.totalCurrentLog());
        m_binder = new Binder(rootLogger, m_machinery);
        m_autons = new Autons(m_machinery);
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_machinery.periodic();
        m_robotLog.periodic();
        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            NetworkTableInstance.getDefault().flush();
        }

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

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
