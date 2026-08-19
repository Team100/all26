package org.team100.frc2026.robot;

import static org.team100.lib.util.TriggerUtil.onTrue;
import static org.team100.lib.util.TriggerUtil.whileTrue;
import static org.wpilib.command2.Commands.parallel;
import static org.wpilib.command2.Commands.sequence;
import static org.wpilib.command2.Commands.waitUntil;

import org.team100.lib.controller.r1.AzimuthController;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.FullStateFeedback;
import org.team100.lib.hid.InterLinkDX;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.subsystems.swerve.commands.manual.DriveFieldRelative;
import org.team100.lib.subsystems.swerve.commands.manual.DriveMovingTargetLock;

/**
 * Binds buttons to commands. Also creates default commands.
 * 
 * See
 * https://docs.google.com/document/d/15HcburjCvwOEBL8ZtQdk-7iotF5qATGK3fO7c5HyWCk
 */
public class Binder {
    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    @SuppressWarnings("unused")
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;

    private final Machinery m_machinery;
    private final LoggerFactory m_log;

    public Binder(Machinery machinery) {
        m_machinery = machinery;
        m_log = rootLogger.name("Commands");

        ////////////////////////////////////////////////////
        ///
        /// CONTROLLER
        ///
        // DriverXboxControl driver = new DriverXboxControl(0);
        InterLinkDX driver = new InterLinkDX(0);

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
                m_machinery.m_intakeExtend.goToRetractedPosition());
        m_machinery.m_shooter.setDefaultCommand(
                m_machinery.m_shooter.stop());
        ////////////////////////////////////////////////////
        ///
        /// DISORIENT
        ///
        /// Back: nudge the rotation towards zero. Start: forget the current pose, listen
        /// to camera input.

        onTrue(driver::back, m_machinery.zeroRotation());
        onTrue(driver::start, m_machinery.disorient());

        ////////////////////////////////////////////////////
        ///
        /// DEFENSE X POSITION
        ///
        whileTrue(driver::povDown, m_machinery.m_drive.defend());

        whileTrue(driver::rightTrigger,
                parallel(
                        m_machinery.m_intakeExtend.goToExtendedPositionEndlessly(),
                        sequence(
                                waitUntil(m_machinery.m_intakeExtend::atGoal),
                                parallel(
                                        m_machinery.m_intake.intake(),
                                        m_machinery.m_shooter.shooterFullspeed()))));

        whileTrue(driver::x,
                m_machinery.m_intake.intake());
        whileTrue(driver::a,
                m_machinery.m_intakeExtend.goToExtendedPositionEndlessly());
        whileTrue(driver::b,
                m_machinery.m_intakeExtend.goToRetractedPosition());
        whileTrue(driver::y, m_machinery.m_shooter.testShooterFullspeed());

        ////////////////////////////////////////////////////
        ///
        /// AIM
        ///
        /// Left bumper: rotate the robot to hit the target

        FeedbackR1 thetaFeedback = new FullStateFeedback(
                m_log, 6, 0.1, true, 0.025, 0.25);

        // button 6
        AzimuthController aim = new AzimuthController(
                m_log,
                m_machinery.m_swerveKinodynamics::getMaxAngleSpeedRad_S,
                thetaFeedback);
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

    }

    /** Keeps tests from conflicting. */
    public void close() {
        //
    }
}
