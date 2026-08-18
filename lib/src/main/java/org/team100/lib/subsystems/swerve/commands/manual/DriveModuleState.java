package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.dynamics.swerve.SwerveEffort;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleState100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The input dtheta is exactly the module angle.
 * The input dx is exactly the wheel speed.
 * The input dy is ignored.
 */
public class DriveModuleState extends Command {
    /**
     * Velocity control in control units, [-1,1] on all axes. This needs to be
     * mapped to a feasible velocity control as early as possible.
     */
    private final Supplier<Velocity> m_twistSupplier;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final DoubleLogger m_log_speed;
    private final Rotation2dLogger m_log_angle;

    public DriveModuleState(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Velocity> twistSupplier,
            SwerveDriveSubsystem drive) {
        LoggerFactory log = parent.type(this);
        m_twistSupplier = twistSupplier;
        m_drive = drive;
        m_swerveKinodynamics = swerveKinodynamics;
        m_log_speed = log.doubleLogger(Level.TRACE, "speed m_s");
        m_log_angle = log.rotation2dLogger(Level.TRACE, "angle rad");
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        Velocity input = m_twistSupplier.get();
        // dtheta is from [-1, 1], so angle is [-pi, pi]
        Optional<Rotation2d> angle = Optional.of(
                Rotation2d.fromRadians(Math.PI * input.theta()));
        double maxSpeed = m_swerveKinodynamics.getMaxDriveVelocityM_S();
        double speedM_S = maxSpeed * input.x();
        m_log_speed.log(() -> speedM_S);
        m_log_angle.log(angle::get);
        SwerveModuleState100 s = new SwerveModuleState100(speedM_S, angle);
        // zero effort for now.
        m_drive.setRawModuleStates(
                new SwerveModuleStates(s, s, s, s), SwerveEffort.ZERO);
    }
}
