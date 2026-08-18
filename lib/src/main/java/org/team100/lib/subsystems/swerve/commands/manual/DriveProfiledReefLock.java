package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.field.FieldConstants2025;
import org.team100.lib.config.DriverSkill;
import org.team100.lib.controller.r1.FeedbackR1;
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
import org.team100.lib.logging.LoggerFactory.ControlR1Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual drivetrain control.
 * 
 * Provides four manual control modes:
 * 
 * -- raw module state
 * -- robot-relative
 * -- field-relative
 * -- field-relative with rotation control
 * 
 * Use the mode supplier to choose which mode to use, e.g. using a Sendable
 * Chooser.
 */
public class DriveProfiledReefLock extends Command {
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

    // don't try to go full speed
    private static final double PROFILE_SPEED = 0.6;
    // accelerate gently to avoid upset
    private static final double PROFILE_ACCEL = 0.5;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Supplier<Translation2d> m_robotLocation;

    /** lock rotation to reef center */
    private final Supplier<Boolean> m_lockToReef;

    private final FeedbackR1 m_thetaFeedback;

    // LOGGERS
    private final BooleanLogger m_log_snap_mode;
    private final DoubleLogger m_log_max_speed;
    private final DoubleLogger m_log_max_accel;
    private final DoubleLogger m_log_goal_theta;
    private final ControlR1Logger m_log_setpoint_theta;
    private final DoubleLogger m_log_theta_FF;
    private final DoubleLogger m_log_theta_FB;
    private final DoubleLogger m_log_output_omega;
    private final LoggerFactory m_log;

    // package private for testing

    private VelocitySE2 m_v;

    ControlR1 m_thetaSetpoint = null;

    public DriveProfiledReefLock(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Boolean> lockToReef,
            FeedbackR1 thetaController,
            Supplier<Translation2d> robotLocation,
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter) {
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_drive = drive;
        m_limiter = limiter;
        m_log = parent.type(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_lockToReef = lockToReef;
        m_robotLocation = robotLocation;
        m_thetaFeedback = thetaController;
        m_log_snap_mode = m_log.booleanLogger(Level.TRACE, "snap mode");
        m_log_max_speed = m_log.doubleLogger(Level.TRACE, "maxSpeedRad_S");
        m_log_max_accel = m_log.doubleLogger(Level.TRACE, "maxAccelRad_S2");
        m_log_goal_theta = m_log.doubleLogger(Level.TRACE, "goal/theta");
        m_log_setpoint_theta = m_log.ControlR1Logger(Level.TRACE, "setpoint/theta");
        m_log_theta_FF = m_log.doubleLogger(Level.TRACE, "thetaFF");
        m_log_theta_FB = m_log.doubleLogger(Level.TRACE, "thetaFB");
        m_log_output_omega = m_log.doubleLogger(Level.TRACE, "output/omega");
        m_v = VelocitySE2.ZERO;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        // make sure the limiter knows what we're doing
        m_limiter.updateSetpoint(new VelocityControlSE2(m_drive.getVelocity()));

        ModelSE2 p = m_drive.getState();

        m_thetaSetpoint = p.theta().control();
        m_thetaFeedback.reset();

    }

    @Override
    public void execute() {

        // input in [-1,1] control units
        Velocity t = m_twistSupplier.get();
        ModelSE2 s = m_drive.getState();

        // scale for driver skill.
        VelocityControlSE2 scaled = GeometryUtil.scale(apply(s, t), DriverSkill.level().scale());

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

    public VelocityControlSE2 apply(
            final ModelSE2 state,
            final Velocity twist1_1) {
        final VelocityControlSE2 control = clipAndScale(twist1_1);

        if (!m_lockToReef.get()) {
            // not locked, just return the input.
            m_thetaSetpoint = null;
            m_log_snap_mode.log(() -> false);
            return control;
        }

        // if this is the first run since the latch, then the setpoint should be
        // whatever the measurement is
        if (m_thetaSetpoint == null) {
            m_thetaSetpoint = state.theta().control();
        }

        // feedback uses the current setpoint, which was set previously
        final double thetaFB = m_thetaFeedback.calculate(state.theta(), m_thetaSetpoint.model());

        final double yawMeasurement = state.theta().x();
        // take the short path
        Rotation2d m_goal = Math100.getMinDistance(
                yawMeasurement,
                FieldConstants2025.angleToReefCenter(m_robotLocation.get()));

        // use the modulus closest to the measurement
        m_thetaSetpoint = new ControlR1(
                Math100.getMinDistance(yawMeasurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());

        final TrapezoidProfileR1 profile = makeProfile(state.velocity().norm());
        m_thetaSetpoint = profile.calculate(
                TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint, new ModelR1(m_goal.getRadians(), 0));

        final double thetaFF = m_thetaSetpoint.v();

        final double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // TODO: accel
        VelocityControlSE2 twistWithSnapM_S = new VelocityControlSE2(
                control.x().v(), control.y().v(), omega);

        m_log_snap_mode.log(() -> true);
        m_log_goal_theta.log(m_goal::getRadians);
        m_log_setpoint_theta.log(() -> m_thetaSetpoint);
        m_log_theta_FF.log(() -> thetaFF);
        m_log_theta_FB.log(() -> thetaFB);
        m_log_output_omega.log(() -> omega);

        return twistWithSnapM_S;
    }

    public VelocityControlSE2 clipAndScale(Velocity twist1_1) {
        // clip the input to the unit circle
        final Velocity clipped = twist1_1.clip(1.0);

        // scale to max in both translation and rotation
        return VelocityControlSE2.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
    }

    /**
     * Note that the max speed and accel are inversely proportional to the current
     * velocity.
     */
    public TrapezoidProfileR1 makeProfile(double currentVelocity) {
        // fraction of the maximum speed
        final double xyRatio = Math.min(1, currentVelocity / m_swerveKinodynamics.getMaxDriveVelocityM_S());
        // fraction left for rotation
        final double oRatio = 1 - xyRatio;
        // add a little bit of default speed
        final double rotationSpeed = Math.max(0.1, oRatio);

        final double maxSpeedRad_S = m_swerveKinodynamics.getMaxAngleSpeedRad_S() * rotationSpeed * PROFILE_SPEED;

        final double maxAccelRad_S2 = m_swerveKinodynamics.getMaxAngleAccelRad_S2() * rotationSpeed * PROFILE_ACCEL;

        m_log_max_speed.log(() -> maxSpeedRad_S);
        m_log_max_accel.log(() -> maxAccelRad_S2);

        return new TrapezoidProfileR1(
                maxSpeedRad_S,
                maxAccelRad_S2,
                0.01);
    }

}
