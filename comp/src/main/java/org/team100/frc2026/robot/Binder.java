package org.team100.frc2026.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static org.team100.lib.util.TriggerUtil.onTrue;
import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.controller.r1.AzimuthController;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.FullStateFeedback;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.commands.manual.DriveFieldRelative;
import org.team100.lib.subsystems.swerve.commands.manual.DriveMovingTargetLock;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Binds buttons to commands. Also creates default commands.
 * 
 * https://docs.google.com/document/d/15HcburjCvwOEBL8ZtQdk-7iotF5qATGK3fO7c5HyWCk
 */
public class Binder {
    private final DriverXboxControl m_driver;

    public Binder(LoggerFactory rootLogger, Machinery machinery) {
        LoggerFactory log = rootLogger.type(this);

        ////////////////////////////////////////////////////
        ///
        /// CONTROLLER
        ///
        m_driver = new DriverXboxControl(log, 0);
        // InterLinkDX driver = new InterLinkDX(m_log, 0);
        SwerveLimiter limiter = new SwerveLimiter(
                log,
                machinery.m_swerveKinodynamics,
                RobotController::getBatteryVoltage);

        ////////////////////////////////////////////////////
        ///
        /// DEFAULT COMMANDS
        ///
        machinery.m_drive.setDefaultCommand(
                new DriveFieldRelative(
                        log,
                        machinery.m_swerveKinodynamics,
                        m_driver::velocity,
                        machinery.m_drive,
                        limiter));
        machinery.m_intake.setDefaultCommand(
                machinery.m_intake.stop());
        machinery.m_intakeExtend.setDefaultCommand(
                machinery.m_intakeExtend.goToRetractedPosition());
        machinery.m_shooter.setDefaultCommand(
                machinery.m_shooter.stop());
        ////////////////////////////////////////////////////
        ///
        /// DISORIENT
        ///
        /// Back: nudge the rotation towards zero.
        /// Start: forget the current pose, listen to camera input.
        ///
        /// both together: warp to the origin. FOR TESTING ONLY.

        onTrue(m_driver::back, machinery.zeroRotation());
        onTrue(m_driver::start, machinery.disorient());
        onTrue(() -> m_driver.start() && m_driver.back(), machinery.zeroPose());

        ////////////////////////////////////////////////////
        ///
        /// DEFENSE X POSITION
        ///
        whileTrue(m_driver::povDown, machinery.m_drive.defend());

        whileTrue(m_driver::rightTrigger,
                parallel(
                        machinery.m_intakeExtend.goToExtendedPositionEndlessly(),
                        sequence(
                                waitUntil(machinery.m_intakeExtend::atGoal),
                                parallel(
                                        machinery.m_intake.intake(),
                                        machinery.m_shooter.shooterFullspeed()))));

        whileTrue(m_driver::x,
                machinery.m_intake.intake());
        whileTrue(m_driver::a,
                machinery.m_intakeExtend.goToExtendedPositionEndlessly());
        whileTrue(m_driver::b,
                machinery.m_intakeExtend.goToRetractedPosition());
        whileTrue(m_driver::y, machinery.m_shooter.testShooterFullspeed());

        ////////////////////////////////////////////////////
        ///
        /// AIM
        ///
        /// Left bumper: rotate the robot to hit the target

        FeedbackR1 thetaFeedback = new FullStateFeedback(
                log, 6, 0.1, true, 0.025, 0.25);

        // button 6
        AzimuthController aim = new AzimuthController(
                log,
                machinery.m_swerveKinodynamics::getMaxAngleSpeedRad_S,
                thetaFeedback);
        whileTrue(() -> m_driver.leftBumper(),
                new DriveMovingTargetLock(
                        log,
                        machinery.m_swerveKinodynamics,
                        aim,
                        m_driver::velocity,
                        limiter,
                        machinery.m_cachedSolution,
                        machinery.m_drive)
                        .withName("Target lock"));

    }

    /** Keeps tests from conflicting. */
    public void close() {
        //
    }
}
