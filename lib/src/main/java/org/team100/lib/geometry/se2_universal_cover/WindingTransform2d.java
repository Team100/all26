package org.team100.lib.geometry.se2_universal_cover;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Rigid-body transform in two dimensions with winding angle, i.e.
 * in the ~SE(2) cover rather than SE(2) itself.
 */
public class WindingTransform2d {
    private final Translation2d m_translation;
    private final double m_rotation;

    public WindingTransform2d(Translation2d t, double r) {
        m_translation = t;
        m_rotation = r;
    }

    public WindingTransform2d(double x, double y, double r) {
        this(new Translation2d(x, y), r);
    }

    public WindingTransform2d(WindingPose2d initial, WindingPose2d last) {
        // To transform the global translation delta to be relative to the initial
        // pose, rotate by the inverse of the initial pose's orientation.
        // it's ok to use SE(2) for the rotation; rotation of a vector
        // in SE(2) and ~SE(2) is the same.
        m_translation = last.getTranslation()
                .minus(initial.getTranslation())
                .rotateBy(new Rotation2d(-1.0 * initial.getRotation()));

        m_rotation = last.getRotation() - initial.getRotation();
    }

    public Translation2d getTranslation() {
        return m_translation;
    }

    public double getRotation() {
        return m_rotation;
    }

    public WindingTwist2d log() {
        final double dtheta = m_rotation;
        final double halfDtheta = dtheta / 2.0;

        final double cosMinusOne = Math.cos(m_rotation) - 1;

        double halfThetaByTanOfHalfDtheta;
        if (Math.abs(cosMinusOne) < 1E-9) {
            halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halfThetaByTanOfHalfDtheta = -(halfDtheta * Math.sin(m_rotation)) / cosMinusOne;
        }

        Translation2d translationPart = m_translation
                .rotateBy(new Rotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta))
                .times(Math.hypot(halfThetaByTanOfHalfDtheta, halfDtheta));

        return new WindingTwist2d(translationPart.getX(), translationPart.getY(), dtheta);
    }

}
