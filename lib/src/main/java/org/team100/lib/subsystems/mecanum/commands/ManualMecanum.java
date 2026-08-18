package org.team100.lib.subsystems.mecanum.commands;

import java.util.function.Supplier;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.mecanum.MecanumDrive100;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.util.EnumChooser;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Map manual velocity input to the Mecanum drive, using some sort of input
 * scaling to fit into its diamond-shaped velocity envelope.
 * 
 * New! Derives acceleration from backwards finite difference.
 */
public class ManualMecanum extends Command {
    public enum InputScaling {
        NONE,
        CLIP,
        SQUASH
    }

    private final Supplier<Velocity> m_velocity;
    private final double m_maxVX;
    private final double m_maxVY;
    private final double m_maxOmega;
    private final SwerveLimiter m_limiter;
    private final MecanumDrive100 m_drive;
    private final EnumChooser<InputScaling> m_chooser;

    private VelocitySE2 m_v;

    public ManualMecanum(
            Supplier<Velocity> velocity,
            double maxVX,
            double maxVY,
            double maxOmega,
            SwerveLimiter limiter,
            MecanumDrive100 drive) {
        if (maxVY > maxVX)
            throw new IllegalArgumentException();
        m_velocity = velocity;
        m_maxVX =  maxVX;
        m_maxVY =  maxVY;
        m_maxOmega =  maxOmega;
        m_limiter = limiter;
        m_drive = drive;
        m_chooser = new EnumChooser<>("Input Scaling", InputScaling.NONE);
        m_v = VelocitySE2.ZERO;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_limiter.updateSetpoint(new VelocityControlSE2(
                m_drive.getState().velocity()));
    }

    @Override
    public void execute() {
        Rotation2d poseRotation = m_drive.getState().rotation();
        // Raw stick input.
        Velocity input = m_velocity.get();
        // Clip the input to the diamond shape.
        double y_x = m_maxVY / m_maxVX;
        Velocity clippedOrSquashed = switch (m_chooser.get()) {
            case NONE -> input;
            case CLIP -> input.diamond(1, y_x, poseRotation);
            case SQUASH -> input.squashedDiamond(1, y_x, poseRotation);
        };
        // Scale stick input to field-relative velocity.
        VelocityControlSE2 scaled = VelocityControlSE2.scale(
                clippedOrSquashed, m_maxVX, m_maxOmega);
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
