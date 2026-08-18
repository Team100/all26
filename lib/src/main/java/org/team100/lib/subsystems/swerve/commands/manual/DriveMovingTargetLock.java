package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.lib.config.DriverSkill;
import org.team100.lib.controller.r1.AzimuthController;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.targeting.CachedSolution;
import org.team100.lib.targeting.Solution;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual cartesian control, with rotational control based on a target position.
 * 
 * Allows moving robot, with fixed target.
 *
 * Rotation uses velocity feedforward and feedback, but no profile, because I
 * think the profile might be a source of noise.
 */
public class DriveMovingTargetLock extends Command {
    /**
     * Pay attention to tags even if they are far away.
     */
    private static final double HEED_RADIUS_M = 6.0;

    private final SwerveKinodynamics m_swerveKinodynamics;
    /**
     * Velocity control in control units, [-1,1] on all axes.
     */
    private final Supplier<Velocity> m_twistSupplier;
    private final DoubleConsumer m_heedRadiusM;
    private final SwerveLimiter m_limiter;
    private final CachedSolution m_solver;
    private final SwerveDriveSubsystem m_drive;

    private final AzimuthController m_aim;
    private final BooleanLogger m_log_aiming;

    private VelocitySE2 m_v;

    public DriveMovingTargetLock(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            AzimuthController aim,
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveLimiter limiter,
            CachedSolution solver,
            SwerveDriveSubsystem drive) {
        LoggerFactory log = parent.type(this);
        m_log_aiming = log.booleanLogger(Level.TRACE, "aiming");
        log.doubleLogger(Level.TRACE, "max omega").log(swerveKinodynamics::getMaxAngleSpeedRad_S);
        m_swerveKinodynamics = swerveKinodynamics;
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_limiter = limiter;
        m_solver = solver;
        m_drive = drive;
        m_aim = aim;
        m_v = VelocitySE2.ZERO;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        m_limiter.updateSetpoint(new VelocityControlSE2(m_drive.getVelocity()));
        m_aim.reset();
    }

    @Override
    public void execute() {
        actuate(getOmega());
    }

    private Double getOmega() {
        Optional<Solution> oSolution = m_solver.get();
        if (oSolution.isEmpty()) {
            // No target, so use the driver input.
            return null;
        }
        Solution solution = oSolution.get();
        ModelR1 target = new ModelR1(
                solution.azimuth().getRadians(),
                solution.azimuthVelocity());
        ModelR1 measurement = m_drive.getState().theta();
        return m_aim.getOmega(measurement, target);
    }

    /** Null to skip override */
    private void actuate(Double omega) {
        // Clip and scale user input.
        VelocityControlSE2 scaled = VelocityControlSE2.scale(
                m_twistSupplier.get().clip(1.0),
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // Scale for driver skill.
        scaled = GeometryUtil.scale(scaled, DriverSkill.level().scale());

        // Apply field-relative limits.
        if (Experiments.instance.enabled(Experiment.UseSwerveLimiter)) {
            scaled = m_limiter.apply(scaled);
        }

        // Override omega.
        m_log_aiming.log(() -> omega != null);
        if (omega != null) {
            scaled = new VelocityControlSE2(scaled.x().v(), scaled.y().v(), omega);
        }

        // Compute field-relative accel from backwards finite difference.
        VelocitySE2 v = scaled.velocity();
        // Because this is field-relative, there is no centrifugal force.
        AccelerationSE2 a = v.accel(m_v, TimedRobot100.LOOP_PERIOD_S);
        m_v = v;

        // Actuate the drivetrain.
        m_drive.set(new VelocityControlSE2(v, a));

    }

}
