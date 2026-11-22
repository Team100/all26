package org.team100.lib.geometry.se2_universal_cover;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.Interpolatable;

/**
 * Like Pose2d, but in the universal cover of SE(2), in which rotation is not
 * periodic, i.e. it "winds". See README.md.
 */
public class WindingPose2d implements Interpolatable<WindingPose2d> {
    public static final WindingPose2d ZERO = new WindingPose2d();

    private final Translation2d m_translation;
    private final double m_rotation;

    public WindingPose2d(Translation2d t, double r) {
        m_translation = t;
        m_rotation = r;
    }

    /** Lift the pose into ~SE(2) using the overlapping sheet */
    public WindingPose2d(Pose2d p) {
        this(p.getTranslation(), p.getRotation().getRadians());
    }

    public WindingPose2d(double x, double y, double r) {
        this(new Translation2d(x, y), r);
    }

    public WindingPose2d() {
        this(Translation2d.ZERO, 0.0);
    }

    /** Project into SE(2) */
    public Pose2d pose2d() {
        return new Pose2d(m_translation, new Rotation2d(m_rotation));
    }

    public Translation2d getTranslation() {
        return m_translation;
    }

    public double getRotation() {
        return m_rotation;
    }

    public double getX() {
        return m_translation.getX();
    }

    public double getY() {
        return m_translation.getY();
    }

    public WindingPose2d transformBy(WindingTransform2d other) {
        return new WindingPose2d(
                m_translation.plus(other.getTranslation().rotateBy(new Rotation2d(m_rotation))),
                other.getRotation() + m_rotation);
    }

    public WindingPose2d relativeTo(WindingPose2d other) {
        WindingTransform2d transform = new WindingTransform2d(other, this);
        return new WindingPose2d(transform.getTranslation(), transform.getRotation());
    }

    public WindingPose2d plus(WindingTransform2d other) {
        return transformBy(other);
    }

    public WindingTransform2d minus(WindingPose2d other) {
        final var pose = this.relativeTo(other);
        return new WindingTransform2d(pose.getTranslation(), pose.getRotation());
    }

    @Override
    public WindingPose2d interpolate(WindingPose2d endValue, double t) {
        if (t < 0) {
            return this;
        } else if (t >= 1) {
            return endValue;
        } else {
            WindingTwist2d twist = endValue.minus(this).log();
            WindingTwist2d scaledTwist = new WindingTwist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
            return this.plus(scaledTwist.exp());
        }
    }

}
