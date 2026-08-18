package org.team100.lib.subsystems.swerve.commands.test;

import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A bang-bang controller that stops after the robot moves the specified
 * distance.
 * 
 * This is for testing odometry.
 */
public class DriveBackwards extends Command {
    private static final ChassisSpeeds SPEED = new ChassisSpeeds(-0.1, 0, 0);

    private final SwerveDriveSubsystem m_drive;
    private final double m_length;

    private Pose2d m_startingPose;

    public DriveBackwards(SwerveDriveSubsystem drive, double length) {
        m_drive = drive;
        m_length = length;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_startingPose = m_drive.getPose();
    }

    @Override
    public void execute() {
        m_drive.setChassisSpeeds(SPEED, ChassisAcceleration.ZERO);
    }

    public boolean isDone() {
        return m_drive.getPose().getTranslation().minus(m_startingPose.getTranslation()).getNorm() >= m_length;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
