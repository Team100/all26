package frc.robot;

import org.team100.lib.subsystems.led.DemoLED;
import org.team100.lib.subsystems.lynxmotion_arm.LynxArmSetup;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.framework.TimedRobot;

public class Robot extends TimedRobot {
    private final DemoLED m_led;
    private final Gamepad m_controller;
    private final Runnable m_setup;

    public Robot() {
        m_led = new DemoLED();
        m_controller = new Gamepad(0);
        // m_led.setDefaultCommand(m_led.sweep());

        /////////////////////////////
        /// 
        /// Choose one of the setups
        /// 

        // calibrate one axis at a time
        // m_setup = new CalibratorSetup(m_controller, m_led);

        // two-dof arm
        // m_setup = new LynxArmTwoDofSetup(m_controller);

        // five-dof arm
        m_setup = new LynxArmSetup(m_controller);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_setup.run();
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
