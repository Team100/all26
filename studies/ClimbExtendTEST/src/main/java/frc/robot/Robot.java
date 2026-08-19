// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.TriggerUtil.whileTrue;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.util.Banner;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.RobotController;
import org.wpilib.system.WPILibVersion;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Machinery m_machinery;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    Banner.printBanner();

    // This is for setting up LaserCAN devices.
    // CanBridge.runTCP();

    System.out.printf("WPILib Version: %s\n", WPILibVersion.Version);
    System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
    System.out.printf("Identity: %s\n", Identity.instance.name());
    RobotController.setBrownoutVoltage(5.5);
    Experiments.instance.show();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_machinery = new Machinery();
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
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
    if (Experiments.instance.enabled(Experiment.FlushOften)) {
      // StrUtil.warn("FLUSHING EVERY LOOP, DO NOT USE IN COMP");
      NetworkTableInstance.getDefault().flush();
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    DriverXboxControl driver = new DriverXboxControl(0);
    whileTrue(driver::x,
        m_machinery.m_ClimberExtension.setPosition());
    whileTrue(driver ::y, m_machinery.m_ClimberExtension.setHomePosition());
    whileTrue(driver:: a, m_machinery.m_Climber.setClimb3());
    whileTrue(driver:: b, m_machinery.m_Climber.setClimb0());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
