package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.util.Startup;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 {

    public Robot() {
        Startup.start();
        LoggerFactory log = Logging.instance().rootLogger;
        new RobotLog(log);

        // CONFIGURATION

        // SUBSYSTEM

        // BINDINGS

    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        // Poll for logs after all the actuation is done
        LogPoller.log();
        if (Experiments.INSTANCE.enabled(Experiment.FlushOften)) {
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

    @Override
    public void close() {
        super.close();
    }

}
