package org.team100.frc2025.setups;

import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.commands.manual.DriveFieldRelative;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;

/**
 * This is the set of manual driving modes we usually support,
 * but we're not using these in 2025, we're using DriveManuallySimple instead.
 */
public class DriveManuallySetup {

    public static void setup(
            LoggerFactory comLog,
            LoggerFactory fieldLog,
            DriverXboxControl driverControl,
            AprilTagRobotLocalizer localizer,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter,
            SwerveKinodynamics swerveKinodynamics,
            FeedbackR1 thetaFeedback) {
        @SuppressWarnings("unused")
        DriveFieldRelative driveManually = new DriveFieldRelative(
                comLog,
                swerveKinodynamics,
                driverControl::velocity,
                drive,
                limiter);
    }
}
