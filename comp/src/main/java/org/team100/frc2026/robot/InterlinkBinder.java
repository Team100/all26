package org.team100.frc2026.robot;

import static org.team100.lib.util.TriggerUtil.onTrue;
import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.controller.r1.AzimuthController;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.hid.InterLinkDX;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.commands.manual.DriveFieldRelative;
import org.team100.lib.subsystems.swerve.commands.manual.DriveMovingTargetLock;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.wpilib.system.RobotController;


/**
 * Control bindings for the Interlink DX. Also default commands.
 * 
 * See https://my.spektrumrc.com/ProdInfo/Files/SPMRFTX1-Manual-EN.pdf
 */
public class InterlinkBinder {

    public InterlinkBinder(LoggerFactory rootLogger, Machinery machinery) {
        LoggerFactory log = rootLogger.name("Commands");

        ////////////////////////////////////////////////////
        ///
        /// CONTROLLER
        ///
        InterLinkDX driver = new InterLinkDX(log, 0);
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
                        driver::velocity,
                        machinery.m_drive,
                        limiter));
        machinery.m_shooter.setDefaultCommand(
                machinery.m_shooter.stop());
        machinery.m_intake.setDefaultCommand(
                machinery.m_intake.stop());
        machinery.m_intakeExtend.setDefaultCommand(
                machinery.m_intakeExtend.stop());

        ////////////////////////////////////////////////////
        ///
        /// DISORIENT
        ///
        onTrue(driver::reset, machinery.disorient());

        ////////////////////////////////////////////////////
        ///
        /// INTAKE
        ///
        whileTrue(driver::c2,
                machinery.m_intakeExtend.goToRetractedPosition());
        whileTrue(driver::c0,
                machinery.m_intakeExtend.goToExtendedPosition()
                        .andThen(machinery.m_intake.intake()));

        ////////////////////////////////////////////////////
        ///
        /// AIM
        ///
        FeedbackR1 thetaFeedback = new PIDFeedback(
                log, 3.2, 0, 0, true, 0.05, 1);

        AzimuthController aim = new AzimuthController(
                log,
                machinery.m_swerveKinodynamics::getMaxAngleSpeedRad_S,
                thetaFeedback);
        whileTrue(() -> driver.a1(),
                new DriveMovingTargetLock(
                        log,
                        machinery.m_swerveKinodynamics,
                        aim,
                        driver::velocity,
                        limiter,
                        machinery.m_cachedSolution,
                        machinery.m_drive)
                        .withName("Target lock"));

        ////////////////////////////////////////////////////
        ///
        /// SHOOT
        ///
        // whileTrue(driver::i,
        // parallel(
        // m_machinery.m_shooter.shooterFullspeed(),
        // Commands.repeatingSequence(
        // waitUntil(m_machinery.m_shooter::atSpeed)
        // .onlyWhile(m_machinery.m_shooter::atSpeed))));

        ////////////////////////////////////////////////////
        ///
        /// TEST
        ///
        // Tester tester = new Tester(m_machinery);
        // whileTrue(() -> (RobotState.isTest() && driver.reset() && driver.cancel()),
        //         tester.prematch());
    }

}
