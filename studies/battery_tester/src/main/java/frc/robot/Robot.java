package frc.robot;

import org.team100.battery_tester.BatteryTester;
import org.team100.battery_tester.ConstantCurrentProtocol;
import org.team100.battery_tester.ConstantPowerProtocol;
import org.team100.battery_tester.LightBulbVisualizer;
import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.util.Banner;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.button.Trigger;
import org.wpilib.system.RobotController;


public class Robot extends TimedRobot100 {
    enum Protocol {
        POWER, CURRENT
    }

    private static final Protocol PROTOCOL = Protocol.CURRENT;

    private final DriverXboxControl m_controller;
    private final BatteryTester m_subsystem;
    private final LightBulbVisualizer m_viz;
    private final Command m_auton;

    public Robot() {
        Banner.printBanner();
        Experiments.instance.show();
        RobotController.setBrownoutVoltage(6.3); // RoboRIO 1.0 value
        Logging log = Logging.instance();
        LoggerFactory robotLog = log.rootLogger;
        m_controller = new DriverXboxControl(0);
        m_subsystem = new BatteryTester(robotLog);
        m_viz = new LightBulbVisualizer(m_subsystem::temperature);
        m_subsystem.setDefaultCommand(m_subsystem.run(m_subsystem::off));
        switch (PROTOCOL) {
            case POWER:
                new Trigger(m_controller::a)
                        .whileTrue(m_subsystem.run(() -> m_subsystem.setPower(100)));
                new Trigger(m_controller::b)
                        .whileTrue(m_subsystem.run(() -> m_subsystem.setPower(500)));
                new Trigger(m_controller::x)
                        .whileTrue(m_subsystem.run(() -> m_subsystem.setPower(1000)));
                // Around 1.5kw is the maximum possible. The battery label capacity is
                // something like 700kJ, so 1.5kw will discharge it fully in about 8 minutes.
                // This will likely destroy the battery; start with something lower.
                new Trigger(m_controller::y)
                        .whileTrue(m_subsystem.run(() -> m_subsystem.setPower(1500)));
                m_auton = new ConstantPowerProtocol(m_subsystem, 1500, 9.5);
                break;
            case CURRENT:
            default:
                new Trigger(m_controller::a)
                        .whileTrue(m_subsystem.run(() -> m_subsystem.setCurrent(50)));
                new Trigger(m_controller::b)
                        .whileTrue(m_subsystem.run(() -> m_subsystem.setCurrent(100)));
                new Trigger(m_controller::x)
                        .whileTrue(m_subsystem.run(() -> m_subsystem.setCurrent(150)));
                new Trigger(m_controller::y)
                        .whileTrue(m_subsystem.run(() -> m_subsystem.setCurrent(200)));
                // this is like the old tester
                m_auton = new ConstantCurrentProtocol(m_subsystem, 10, 10.5);
                // 
                // m_auton = new ConstantCurrentProtocol(m_subsystem, 150, 9.5);
                break;
        }
    }

    void setupConstantPower() {

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
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(m_auton);
    }

    @Override
    public void autonomousPeriodic() {
        //
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {
        //
    }

    @Override
    public void simulationPeriodic() {
        //
    }
}
