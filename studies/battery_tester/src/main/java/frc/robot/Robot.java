
package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.util.Banner;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.button.Trigger;
import org.wpilib.framework.TimedRobot;
import org.wpilib.system.RobotController;

public class Robot extends TimedRobot {

    private final DriverXboxControl m_controller;
    private final BatteryTester m_subsystem;
    private final LightBulbVisualizer m_viz;

    public Robot() {
        Banner.printBanner();
        Experiments.instance.show();
        RobotController.setBrownoutVoltage(5.0);
        Logging log = Logging.instance();
        LoggerFactory robotLog = log.rootLogger;
        m_controller = new DriverXboxControl(0);
        m_subsystem = new BatteryTester(robotLog);
        m_viz = new LightBulbVisualizer(m_subsystem::temperature);
        new Trigger(m_controller::a)
                .whileTrue(m_subsystem.run(() -> m_subsystem.setPower(100)));
        new Trigger(m_controller::b)
                .whileTrue(m_subsystem.run(() -> m_subsystem.setPower(500)));
        new Trigger(m_controller::x)
                .whileTrue(m_subsystem.run(() -> m_subsystem.setPower(1000)));
        // Around 1.5kw is the maximum possible.  The battery label capacity is
        // something like 700kJ, so 1.5kw will discharge it fully in about 8 minutes.
        // This will likely destroy the battery; start with something lower.
        new Trigger(m_controller::y)
                .whileTrue(m_subsystem.run(() -> m_subsystem.setPower(1500)));
        m_subsystem.setDefaultCommand(m_subsystem.run(m_subsystem::off));
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_viz.periodic();
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
