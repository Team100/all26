package frc.robot;

import static frc.robot.TriggerUtil.whileTrue;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.util.Startup;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final Machinery m_machinery;
    private final Logging logging = Logging.instance();
    private final LoggerFactory logger = logging.rootLogger;
    private final DriverXboxControl driver = new DriverXboxControl(logger, 0);

    public Robot() {
        Startup.start();
        m_machinery = new Machinery();
    }

    @Override
    public void robotPeriodic() {
        // Advance the drumbeat.
        Takt.update();
        // reply to sync requests.

        // Take all the measurements we can, as soon and quickly as possible.
        Cache.refresh();
        // Run one iteration of the command scheduler.
        CommandScheduler.getInstance().run();
        m_machinery.periodic();
        // Poll for logs after all the actuation is done
        LogPoller.log();
        if (Experiments.INSTANCE.enabled(Experiment.FlushOften)) {
            // StrUtil.warn("FLUSHING EVERY LOOP, DO NOT USE IN COMP");
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    public void teleopInit() {
        whileTrue(driver::x,
                m_machinery.m_ClimberExtension.setPosition());
        whileTrue(driver::y, m_machinery.m_ClimberExtension.setHomePosition());
        whileTrue(driver::a, m_machinery.m_Climber.setClimb3());
        whileTrue(driver::b, m_machinery.m_Climber.setClimb0());
    }

}
