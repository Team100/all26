package org.team100.frc2025.Swerve;

import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForwardSlowly extends Command {
    private final SwerveDriveSubsystem m_drive;

    public DriveForwardSlowly(SwerveDriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.setChassisSpeeds(
                new ChassisSpeeds(0, 0.1, 0), ChassisAcceleration.ZERO);
    }
}
