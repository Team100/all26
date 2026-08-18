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
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Function that supports manual cartesian control, and both manual and locked
 * rotational control.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * The profile depends on robot speed, making rotation the lowest priority.
 */
public class DriveBargeAssist extends Command {
    /**
     * x coordinate of the barge scoring location
     */
    private static final double BARGE_X = 7.4;
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

    private final Supplier<Pose2d> m_pose;
    private VelocitySE2 m_v;

    public DriveBargeAssist(
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Pose2d> pose,
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter) {
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_drive = drive;
        m_limiter = limiter;
        m_swerveKinodynamics = swerveKinodynamics;
        m_pose = pose;
        m_v = VelocitySE2.ZERO;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        // make sure the limiter knows what we're doing
        m_limiter.updateSetpoint(new VelocityControlSE2(m_drive.getVelocity()));

    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds.
     * 
     * If you touch the POV and not the twist rotation, it remembers the POV. if you
     * use the twist rotation, it forgets and just uses that.
     * 
     * Desaturation prefers the rotational profile completely in the snap case, and
     * normally in the non-snap case.
     */
    @Override
    public void execute() {

        // input in [-1,1] control units
        Velocity input = m_twistSupplier.get();

        // clip the input to the unit circle
        Velocity clipped = input.clip(1.0);

        Velocity avoidBarge = avoidBarge(clipped);

        VelocityControlSE2 scaled = VelocityControlSE2.scale(
                avoidBarge,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // scale for driver skill.
        scaled = GeometryUtil.scale(scaled, DriverSkill.level().scale());

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

    private Velocity avoidBarge(Velocity clipped) {
        if (clipped.x() <= 0)
            return clipped;
        // moving towards the barge
        double distance = BARGE_X - m_pose.get().getX();
        double vx = vx(distance);
        return new Velocity(vx, clipped.y(), clipped.theta());
    }

    private double vx(double distance) {
        if (Math.abs(distance) < 0.01) {
            return 0;
        }
        return Math.max(0.5, distance * distance);

    }
}
