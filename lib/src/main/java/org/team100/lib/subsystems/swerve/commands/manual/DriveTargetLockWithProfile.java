
package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

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
import org.team100.lib.logging.LoggerFactory.ControlR1Logger;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.ModelR1Logger;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.targeting.TargetUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual cartesian control, with rotational control based on a target position.
 * 
 * This is useful for shooting solutions, or for keeping the camera pointed at
 * something.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * The targeting solution is based on bearing alone, so it won't work if the
 * robot or target is moving. That effect can be compensated, though.
 * 
 * I think the profile might be a source of noise when the setpoint and the
 * measurement are close to each other.
 */
public class DriveTargetLockWithProfile extends Command {
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

    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double ROTATION_SPEED = 0.5;

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Supplier<Translation2d> m_target;
    private final FeedbackR1 m_thetaController;
    private final ProfileR1 m_profile;

    private final DoubleLogger m_log_apparent_motion;
    private final DoubleArrayLogger m_log_target;
    private final ControlR1Logger m_log_setpoint;
    private final ModelR1Logger m_log_goal;
    private final DoubleLogger m_log_thetaFB;
    private final DoubleLogger m_log_thetaFF;

    private ControlR1 m_thetaSetpoint;
    private VelocitySE2 m_v;

    public DriveTargetLockWithProfile(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Translation2d> target,
            FeedbackR1 thetaController,
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter) {
        LoggerFactory log = parent.type(this);
        m_log_setpoint = log.ControlR1Logger(Level.TRACE, "setpoint");
        m_log_goal = log.ModelR1Logger(Level.TRACE, "goal");
        m_log_thetaFB = log.doubleLogger(Level.TRACE, "thetaFB");
        m_log_thetaFF = log.doubleLogger(Level.TRACE, "thetaFF");
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_drive = drive;
        m_limiter = limiter;
        m_log_target = fieldLogger.doubleArrayLogger(Level.TRACE, "target");
        m_swerveKinodynamics = swerveKinodynamics;
        m_target = target;
        m_thetaController = thetaController;
        m_profile = new TrapezoidProfileR1(
                swerveKinodynamics.getMaxAngleSpeedRad_S() * ROTATION_SPEED,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * ROTATION_SPEED,
                0.01);
        m_log_apparent_motion = log.doubleLogger(Level.TRACE, "apparent motion");
        m_v = VelocitySE2.ZERO;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        m_limiter.updateSetpoint(new VelocityControlSE2(m_drive.getVelocity()));
        ModelSE2 state = m_drive.getState();
        // always use zero initial setpoint velocity to avoid "jerk" on init.
        // TODO: is this ok?
        // m_thetaSetpoint = p.theta().control();
        m_thetaSetpoint = new ControlR1(state.theta().x(), 0);
        m_thetaController.reset();
    }

    @Override
    public void execute() {
        ModelSE2 state = m_drive.getState();

        // Feedback based on the current state and the previous setpoint.
        double thetaFB = m_thetaController.calculate(state.theta(), m_thetaSetpoint.model());
        m_log_thetaFB.log(() -> thetaFB);

        // the goal omega should match the target's apparent motion
        Translation2d target = m_target.get();
        m_log_target.log(() -> new double[] {
                target.getX(),
                target.getY(),
                0 });
        double targetMotion = TargetUtil.targetMotion(state, target);
        m_log_apparent_motion.log(() -> targetMotion);

        double unwrappedBearing = TargetUtil.unwrappedAbsoluteBearing(state.pose(), target);
        // eliminate target motion to reduce noise
        // TODO: put back target motion
        ModelR1 goal = new ModelR1(unwrappedBearing, 0);
        // final ModelR1 goal = new ModelR1(unwrappedBearing, targetMotion);
        m_log_goal.log(() -> goal);

        // Make sure the old setpoint uses the modulus close to the measurement.
        m_thetaSetpoint = new ControlR1(
                Math100.getMinDistance(state.theta().x(), m_thetaSetpoint.x()),
                m_thetaSetpoint.v());

        // New setpoint for the next timestep.
        m_thetaSetpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint, goal);
        m_log_setpoint.log(() -> m_thetaSetpoint);

        // Feedforward is for the next time step.
        double thetaFF = m_thetaSetpoint.v();
        m_log_thetaFF.log(() -> thetaFF);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

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
        // TODO: remember accel
        scaled = new VelocityControlSE2(scaled.x().v(), scaled.y().v(), omega);

        // Compute field-relative accel from backwards finite difference.
        VelocitySE2 v = scaled.velocity();
        // Because this is field-relative, there is no centrifugal force.
        AccelerationSE2 a = v.accel(m_v, TimedRobot100.LOOP_PERIOD_S);
        m_v = v;

        // Actuate the drivetrain.
        m_drive.set(new VelocityControlSE2(v, a));
    }

}
