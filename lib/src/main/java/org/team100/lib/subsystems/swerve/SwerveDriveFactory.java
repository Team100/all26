package org.team100.lib.subsystems.swerve;

import org.team100.lib.localization.AprilTagCornerRobotLocalizer;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;

/**
 * Pull together some of the drivetrain's dependencies so they don't pollute
 * Robot.java.
 */
public class SwerveDriveFactory {

    public static SwerveDriveSubsystem get(
            LoggerFactory driveLog,
            SwerveKinodynamics swerveKinodynamics,
            // AprilTagRobotLocalizer localizer,
            AprilTagCornerRobotLocalizer localizer,
            OdometryUpdater odometryUpdater,
            SwerveHistory history,
            SwerveModuleCollection modules) {

        FreshSwerveEstimate estimate = new FreshSwerveEstimate(
                localizer::update,
                odometryUpdater::update,
                history);
        SwerveLocal swerveLocal = new SwerveLocal(
                driveLog,
                swerveKinodynamics,
                modules);
        return new SwerveDriveSubsystem(
                driveLog,
                odometryUpdater,
                estimate,
                swerveLocal);
    }

}
