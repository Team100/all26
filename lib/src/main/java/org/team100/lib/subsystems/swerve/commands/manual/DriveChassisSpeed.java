
package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.function.Supplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The twist components, x, y, and theta, are mapped directly to the
 * corresponding ChassisSpeeds components (and scaled).
 */
public class DriveChassisSpeed extends Command {

    private final Supplier<Velocity> m_twistSupplier;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final ChassisSpeedsLogger m_log_chassis_speeds;
    private ChassisSpeeds m_speed;

    public DriveChassisSpeed(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Velocity> twistSupplier,
            SwerveDriveSubsystem drive) {
        LoggerFactory log = parent.type(this);
        m_twistSupplier = twistSupplier;
        m_drive = drive;
        m_log_chassis_speeds = log.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
        m_swerveKinodynamics = swerveKinodynamics;
        m_speed = new ChassisSpeeds();
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        Velocity clipped = m_twistSupplier.get().clip(1.0);
        double maxSpeed = m_swerveKinodynamics.getMaxDriveVelocityM_S();
        double maxOmega = m_swerveKinodynamics.getMaxAngleSpeedRad_S();
        ChassisSpeeds scaled = new ChassisSpeeds(
                maxSpeed * MathUtil.clamp(clipped.x(), -1, 1),
                maxSpeed * MathUtil.clamp(clipped.y(), -1, 1),
                maxOmega * MathUtil.clamp(clipped.theta(), -1, 1));
        m_log_chassis_speeds.log(() -> scaled);
        ChassisAcceleration accel = ChassisAcceleration.diff(
                scaled, m_speed, TimedRobot100.LOOP_PERIOD_S);
        m_speed = scaled;
        m_drive.setChassisSpeeds(scaled, accel);
    }
}
