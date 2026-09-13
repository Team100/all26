package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.subsystems.discus.setups.SetupBare;
import org.team100.lib.subsystems.discus.setups.SetupMech;
import org.team100.lib.subsystems.discus.setups.SetupServo;
import org.team100.lib.util.Startup;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.networktables.NetworkTableInstance;

public class Robot extends TimedRobot100 {
    private enum Setup {
        /** Low-level control */
        BARE,
        /** Adds gearing, friction, and PID */
        MECH,
        /** Adds profiled motion */
        SERVO
    }

    /** Choose one of the setups. */
    private static final Setup SETUP = Setup.BARE;

    private final RobotLog m_robotLog;

    private final Runnable m_setup;

    /** Mirrors the comp robot code. */
    public Robot() {
        Startup.start();
        LoggerFactory rootLogger = Logging.instance().rootLogger;
        m_robotLog = new RobotLog(rootLogger);
        m_setup = switch (SETUP) {
            case BARE -> new SetupBare(m_robotLog.totalCurrentLog());
            case MECH -> new SetupMech(m_robotLog.totalCurrentLog());
            case SERVO -> new SetupServo(m_robotLog.totalCurrentLog());
        };
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_setup.run();
        m_robotLog.periodic();

        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            NetworkTableInstance.getDefault().flush();
        }
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
