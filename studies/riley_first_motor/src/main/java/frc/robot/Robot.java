// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.rev.MinionSparkMotor;
import org.team100.lib.util.CanId;
import org.team100.lib.util.Startup;
import java.util.concurrent.TimeUnit;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot100 {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
private final MinionSparkMotor m_drumA;
private final MinionSparkMotor m_drumB;
private final MinionSparkMotor m_indexer;
private double m_dutyCycle1 = 0;
private double m_dutyCycle2 = 0;
private double m_dutyCycle3 = 0;
private DriverXboxControl m_controller;


public Robot() {
    Startup.start();
    LoggerFactory log = Logging.root();
    RobotLog robotLog = new RobotLog(log);
    TotalCurrentLog currentLog = robotLog.totalCurrentLog();
    m_controller = new DriverXboxControl(log, 0);
  


    System.out.println("Initializing motor");
    m_drumA = new MinionSparkMotor(
            log.name("drum A"),
            currentLog,
            new CanId(39), 
            NeutralMode100.COAST,
            MotorPhase.FORWARD,
            new CurrentLimit(10, 10),
            new Friction(0, 0, 0, 0),
            PIDConstants.zero(),
            0,
            0);
    m_drumB = new MinionSparkMotor(
            log.name("drum B"),
            currentLog,
            new CanId(8), 
            NeutralMode100.COAST,
            MotorPhase.FORWARD,
            new CurrentLimit(10, 10),
            new Friction(0, 0, 0, 0),
            PIDConstants.zero(),
            0,
            0);
    m_indexer = new MinionSparkMotor(
            log.name("indexer"),
            currentLog,
            new CanId(7), 
            NeutralMode100.COAST,
            MotorPhase.FORWARD,
            new CurrentLimit(10, 10),
            new Friction(0, 0, 0, 0),
            PIDConstants.zero(),
            0,
            0);
    System.out.println("Motor initialized");

}



  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
            Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();

        LogPoller.log();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
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
    m_dutyCycle1 = -0.3;
    m_dutyCycle2 = 0.3;
    m_dutyCycle3 = 0;
    m_drumA.setDutyCycle(m_dutyCycle1);
    m_drumB.setDutyCycle(m_dutyCycle2);
    m_indexer.setDutyCycle(m_dutyCycle3);
    try {
    // Pauses the thread for exactly 4 seconds
      TimeUnit.SECONDS.sleep(4); 
      System.out.println("Motors powered");
    } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
    }
    m_dutyCycle1 = -0.3;
    m_dutyCycle2 = 0.3;
    m_dutyCycle3 = -0.3;
    m_drumA.setDutyCycle(m_dutyCycle1);
    m_drumB.setDutyCycle(m_dutyCycle2);
    m_indexer.setDutyCycle(m_dutyCycle3);
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // double m_speed = m_drumA.encoder().getVelocityRad_S();
    // System.out.printf("m_speed %.2f%n", m_speed);

    // Slowly speeds up
    // if(m_dutyCycle < 0.3){
    //   m_dutyCycle += 0.001;
    // }
    // m_motor.setDutyCycle(m_dutyCycle);
    // controlled by controller
    // m_dutyCycle = m_controller.leftY() / 10.0;

    m_dutyCycle1 = -0.3;
    m_dutyCycle2 = 0.3;
    m_dutyCycle3 = -0.3;
    m_drumA.setDutyCycle(m_dutyCycle1);
    m_drumB.setDutyCycle(m_dutyCycle2);
    m_indexer.setDutyCycle(m_dutyCycle3);
    try {
    // Pauses the thread for exactly 1 seconds
      Thread.sleep(500); 
      System.out.println("Motors shooting");
    } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
    }
    
    m_dutyCycle1 = -0.3;
    m_dutyCycle2 = 0.3;
    m_dutyCycle3 = 0;
    m_drumA.setDutyCycle(m_dutyCycle1);
    m_drumB.setDutyCycle(m_dutyCycle2);
    m_indexer.setDutyCycle(m_dutyCycle3);

    try {
    // Pauses the thread for exactly 2 seconds
      TimeUnit.SECONDS.sleep(2); 
      System.out.println("Motors repowered");
    } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
    }
    m_dutyCycle1 = -0.3;
    m_dutyCycle2 = 0.3;
    m_dutyCycle3 = -0.3;
    m_drumA.setDutyCycle(m_dutyCycle1);
    m_drumB.setDutyCycle(m_dutyCycle2);
    m_indexer.setDutyCycle(m_dutyCycle3);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
