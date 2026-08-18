package org.team100.frc2026.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static org.team100.lib.util.TriggerUtil.onTrue;
import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.frc2026.auton.CenterFullSweepAuton;
import org.team100.frc2026.auton.CenterHalfSweepAuton;
import org.team100.frc2026.auton.RightBumpFullSweepAuton;
import org.team100.frc2026.auton.RightBumpHalfSweepAuton;
import org.team100.lib.controller.r1.AzimuthController;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.FullStateFeedback;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.subsystems.swerve.commands.manual.DriveFieldRelative;
import org.team100.lib.subsystems.swerve.commands.manual.DriveMovingTargetLock;

import edu.wpi.first.wpilibj.RobotState;

/**
 * This is a version from pre-SVR testing
 */
public class TestBinder {
    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    @SuppressWarnings("unused")
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;

    private final Machinery m_machinery;
    private final LoggerFactory m_log;

    public TestBinder(Machinery machinery) {
        m_machinery = machinery;
        m_log = rootLogger.name("Commands");

        ////////////////////////////////////////////////////
        ///
        /// CONTROLLER
        ///
        DriverXboxControl driver = new DriverXboxControl(0);

        ////////////////////////////////////////////////////
        ///
        /// DEFAULT COMMANDS
        ///
        m_machinery.m_drive.setDefaultCommand(
                new DriveFieldRelative(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        driver::velocity,
                        m_machinery.m_localizer::setHeedRadiusM,
                        m_machinery.m_drive,
                        m_machinery.m_limiter));
        m_machinery.m_intake.setDefaultCommand(
                m_machinery.m_intake.stop());
        m_machinery.m_intakeExtend.setDefaultCommand(
                m_machinery.m_intakeExtend.stop());
        m_machinery.m_shooter.setDefaultCommand(
                m_machinery.m_shooter.stop());
        

        ////////////////////////////////////////////////////
        ///
        /// DISORIENT
        ///

        // Forget the current pose, listen to camera input.
        onTrue(driver::back, m_machinery.disorient());
        // Nudge the rotation towards zero.
        onTrue(driver::start, m_machinery.zeroRotation());

        ////////////////////////////////////////////////////
        ///
        /// TEST/DEV
        ///
        // This is for testing pose estimation accuracy and drivetrain positioning
        // accuracy.
        // HolonomicProfile profile = HolonomicProfileFactory.get(
        // m_log, m_machinery.m_swerveKinodynamics, 1, 0.5, 1, 0.2);
        // onTrue(driver::b,
        // new DriveToPoseWithProfile(
        // m_log, m_machinery.m_drive, m_machinery.m_holonomicController,
        // profile, () -> new Pose2d(15.387, 3.501, new Rotation2d(0))));

        ////////////////////////////////////////////////////
        ///
        /// CLIMBER
        ///

        // whileTrue(driver::x,
        // m_machinery.m_ClimberExtension.setPosition()
        // .andThen(m_machinery.m_Climber.setClimb1()));
        // whileTrue(driver::a,
        // sequence(
        // m_machinery.m_ClimberExtension.setPosition().withTimeout(1),
        // m_machinery.m_Climber.setClimb3().withTimeout(1)));

        // whileTrue(driver::y,
        // m_machinery.m_Climber.setClimb0()
        // .andThen(m_machinery.m_ClimberExtension.setHomePosition()));

        // These are from ClimberExtendTEST
        // whileTrue(driver::x, m_machinery.m_ClimberExtension.setPosition());
        // whileTrue(driver::y, m_machinery.m_ClimberExtension.setHomePosition());
        // whileTrue(driver::rightTrigger,
        // m_machinery.m_ClimberExtension.setPosition());
        // whileTrue(driver::a, m_machinery.m_Climber.setClimb3());
        // whileTrue(driver::b, m_machinery.m_Climber.setClimb0());

        ////////////////////////////////////////////////////
        ///
        /// INTAKE
        ///

        whileTrue(driver::rightBumper,
                m_machinery.m_intakeExtend.goToRetractedPosition());
        whileTrue(driver::rightTrigger,
                m_machinery.m_intakeExtend.goToExtendedPosition()
                        .andThen(m_machinery.m_intake.intake()));

        // For testing
        // whileTrue(driver::leftBumper,
        // m_machinery.m_intakeExtender.goToExtendedPosition());
        // whileTrue(driver::rightBumper,
        // m_machinery.m_intakeExtender.goToRetractedPosition());

        ////////////////////////////////////////////////////
        ///
        /// AIM
        ///

        FeedbackR1 aggressiveFeedback = new FullStateFeedback(
                m_log, 3, 0.1, true, 0.025, 0.25);

        // button 6
        AzimuthController aim = new AzimuthController(
                m_log,
                m_machinery.m_swerveKinodynamics::getMaxAngleSpeedRad_S,
                aggressiveFeedback);
        whileTrue(() -> driver.leftBumper(),
                new DriveMovingTargetLock(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        aim,
                        driver::velocity,
                        m_machinery.m_localizer::setHeedRadiusM,
                        m_machinery.m_limiter,
                        m_machinery.m_cachedSolution,
                        m_machinery.m_drive)
                        .withName("Target lock"));

        ////////////////////////////////////////////////////
        ///
        /// SHOOT
        ///

        whileTrue(driver::leftTrigger,
                parallel(
                        m_machinery.m_shooter.auto(),
                        repeatingSequence(
                                waitUntil(
                                        m_machinery.m_shooter::atSpeed)
                                        .onlyWhile(m_machinery.m_shooter::atSpeed))));

        //////////////////
        ///
        /// SHOOTER TESTING
        ///

        // whileTrue(driver::x, m_machinery.m_shooter.shooterFullspeed());
        // whileTrue(driver::x, m_machinery.m_shooter.testMotor1Command());
        // whileTrue(driver::y, m_machinery.m_shooter.testMotor2Command());
        // whileTrue(driver::a, m_machinery.m_shooter.testMotor3Command());
        // whileTrue(driver::b, parallel(runShooter, runSerial, runSerialUpper));

        // for friction and feedforward testing

        // whileTrue(driver::a,
        // m_machinery.m_intakeExtend.setVelocity(1));
        // whileTrue(driver::a,
        // m_machinery.m_intakeExtend.setPosition(3));
        // whileTrue(driver::a,
        // m_machinery.m_intake.setVelocity(5));
        // whileTrue(driver::a,
        // m_machinery.m_conveyor.setVelocity(2));
        // whileTrue(driver::a,
        // m_machinery.m_feeder.setVelocity(2));
        // whileTrue(driver::a,
        // m_machinery.m_shooter.setVelocity(15));
        // whileTrue(driver::a,
        // m_machinery.m_shooterHood.setVelocity(1));
        // whileTrue(driver::a,
        // m_machinery.m_shooterHood.setPosition(0.4));
        // whileTrue(driver::b,
        // m_machinery.m_shooterHood.setPosition(0));

        // whileTrue(driver::b,
        // parallel(
        // m_machinery.m_conveyor.testConveyorBack(),
        // m_machinery.m_feeder.testFeedBack()));
        whileTrue(driver::x,
                m_machinery.m_shooter.testRun());

        // whileTrue(driver::rightTrigger, parallel(runSerial, runSerialUpper,
        // runShooter));

        ////////////////////////////////////////////////////
        ///
        /// AUTON TESTING
        ///
        /// Auton test mode is with POV down.

        whileTrue(() -> driver.povDown() && driver.a(),
                new CenterFullSweepAuton(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        m_machinery.m_holonomicController,
                        m_machinery).command());

        whileTrue(() -> driver.povDown() && driver.b(),
                new CenterHalfSweepAuton(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        m_machinery.m_holonomicController,
                        m_machinery).command());
        whileTrue(() -> driver.povDown() && driver.x(),
                new RightBumpFullSweepAuton(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        m_machinery.m_holonomicController,
                        m_machinery).command());
        whileTrue(() -> driver.povDown() && driver.y(),
                new RightBumpHalfSweepAuton(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        m_machinery.m_holonomicController,
                        m_machinery).command());

        ////////////////////////////////////////////////////
        ///
        /// TEST
        ///

        // if (m_machinery.m_intakeExtend.atExtendedPosition()) {
        // whileTrue(driver::y,
        // Commands.repeatingSequence(runIntakeWobbleExtendOut.withTimeout(0.5)
        // .andThen(runIntakeWobbleRetractOut).withTimeout(0.5)));
        // };

        // whileTrue(driver::y,
        //         repeatingSequence(
        //                 m_machinery.m_intakeExtend.goToWobbleSlightlyInExtendedPosition().withTimeout(0.5),
        //                 m_machinery.m_intakeExtend.goToWobbleSlightlyOutRetractedPosition().withTimeout(0.5)));

        whileTrue(driver::povUp,(
                m_machinery.m_shooter.tune()));

        Tester tester = new Tester(m_machinery);
        onTrue(() -> RobotState.isTest(), tester.prompt());
        whileTrue(() -> (RobotState.isTest() && driver.a() && driver.b()),
                tester.prematch());
    }

    /** Keeps tests from conflicting. */
    public void close() {
        //
    }
}
