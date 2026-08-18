package org.team100.lib.geometry.se2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * This is just a container for the difference between two poses.
 * 
 * This treats the dimensions as independent, i.e. in the R3 tangent space,
 * not the SE(2) manifold. Actually it's more like R2xS1, independently.
 * 
 * The SE(2) difference represents a *geodesic* in SE(2), and for differences
 * that include rotation, this will appear as a curved path -- usually not what
 * is desired.
 * 
 * The R3 difference represents a straight line in every axis.
 * 
 * This is useful for control problems where the dimensions are treated as
 * independent, e.g. if you have three separate proportional feedback
 * controllers, or if you want to interpolate the axes separately.
 */
public class DeltaSE2 {
    private final Translation2d m_translation;
    private final Rotation2d m_rotation;

    public DeltaSE2(Translation2d translation, Rotation2d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    @Override
    public String toString() {
        return String.format("%6.3f, %6.3f, %6.3f",
                getX(), getY(), getRotation().getRadians());
    }

    /** Return a delta from start to end. Wraps heading. */
    public static DeltaSE2 delta(Pose2d start, Pose2d end) {
        Translation2d t = end.getTranslation().minus(start.getTranslation());
        Rotation2d r = end.getRotation().minus(start.getRotation());
        return new DeltaSE2(t, r);
    }

    /** Add this delta to the supplied pose. */
    public Pose2d plus(Pose2d start) {
        return new Pose2d(
                start.getTranslation().plus(m_translation),
                start.getRotation().plus(m_rotation));
    }

    public double l2Norm() {
        return Math.sqrt(m_translation.getX() * m_translation.getX()
                + m_translation.getY() * m_translation.getY()
                + m_rotation.getRadians() * m_rotation.getRadians());
    }

    public DeltaSE2 limit(double cartesian, double rotation) {
        return new DeltaSE2(
                new Translation2d(
                        MathUtil.clamp(m_translation.getX(), -cartesian, cartesian),
                        MathUtil.clamp(m_translation.getY(), -cartesian, cartesian)),
                new Rotation2d(
                        MathUtil.clamp(m_rotation.getRadians(), -rotation, rotation)));
    }

    public DeltaSE2 times(double scalar) {
        return new DeltaSE2(m_translation.times(scalar), m_rotation.times(scalar));
    }

    public DeltaSE2 div(double scalar) {
        return new DeltaSE2(m_translation.div(scalar), m_rotation.div(scalar));
    }

    public double getX() {
        return m_translation.getX();
    }

    public double getY() {
        return m_translation.getY();
    }

    public Translation2d getTranslation() {
        return m_translation;
    }

    public Rotation2d getRotation() {
        return m_rotation;
    }

    public double getRadians() {
        return m_rotation.getRadians();
    }

}
