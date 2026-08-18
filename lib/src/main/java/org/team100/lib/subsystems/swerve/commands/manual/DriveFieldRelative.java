package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.lib.config.DriverSkill;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.VelocityControlSE2Logger;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveFieldRelative extends Command {
    /**
     * While driving manually, pay attention to tags even if they are somewhat far
     * away.
     */
    private static final double HEED_RADIUS_M = 6.0;

    /**
     * Velocity control in control units, [-1,1] on all axes. This needs to be
     * mapped to a feasible velocity control as early as possible.
     */
    private final Supplier<Velocity> m_twistSupplier;
    private final DoubleConsumer m_heedRadiusM;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveLimiter m_limiter;

    private final SwerveKinodynamics m_swerveKinodynamics;
    // LOGGERS
    private final VelocityControlSE2Logger m_log_scaled;

    private VelocitySE2 m_v;

    public DriveFieldRelative(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter) {
        LoggerFactory log = parent.type(this);
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_drive = drive;
        m_limiter = limiter;
        m_log_scaled = log.velocityControlSE2Logger(Level.TRACE, "scaled");
        m_swerveKinodynamics = swerveKinodynamics;
        m_v = VelocitySE2.ZERO;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        // make sure the limiter knows what we're doing
        m_limiter.updateSetpoint(new VelocityControlSE2(m_drive.getVelocity()));
    }

    @Override
    public void execute() {
        Velocity clipped = m_twistSupplier.get().clip(1.0);
        VelocityControlSE2 scaled1 = VelocityControlSE2.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        m_log_scaled.log(() -> scaled1);

        // scale for driver skill.
        VelocityControlSE2 scaled = GeometryUtil.scale(scaled1, DriverSkill.level().scale());
        // Apply field-relative limits.

        if (Experiments.instance.enabled(Experiment.UseSwerveLimiter)) {
            scaled = m_limiter.apply(scaled);
        }
        // Compute field-relative accel from backwards finite difference.
        VelocitySE2 v = scaled.velocity();
        // Because this is field-relative, there is no centrifugal force.
        AccelerationSE2 a = v.accel(m_v, TimedRobot100.LOOP_PERIOD_S);
        m_v = v;
        m_drive.set(new VelocityControlSE2(v, a));
    }

}
