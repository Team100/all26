package org.team100.lib.uncertainty;

import edu.wpi.first.math.MathUtil;

/**
 * Represents measurement and estimate uncertainty in the situation we actually
 * have, where x and y are equivalent in every respect, and the covariances in
 * SE(2) are assumed to be zero for simplicity. This leaves only two numbers.
 */
public class IsotropicNoiseSE2 {
    // we store variance to avoid all those squares and square roots.
    private final double m_cartesianVariance;
    private final double m_rotationVariance;

    private IsotropicNoiseSE2(double cartesianVariance, double rotationVariance) {
        m_cartesianVariance = cartesianVariance;
        m_rotationVariance = rotationVariance;
    }

    /**
     * @param cartesianStdDev Standard deviation of cartesian dimensions.
     * @param rotationStdDev  Standard deviation of rotation.
     */
    public static IsotropicNoiseSE2 fromStdDev(double cartesianStdDev, double rotationStdDev) {
        return new IsotropicNoiseSE2(
                Math.pow(cartesianStdDev, 2),
                Math.pow(rotationStdDev, 2));
    }

    public static IsotropicNoiseSE2 fromVariance(double cartesian, double rotation) {
        return new IsotropicNoiseSE2(cartesian, rotation);
    }

    /** Effectively infinite uncertainty. */
    public static IsotropicNoiseSE2 high() {
        return IsotropicNoiseSE2.fromStdDev(10, 6);
    }

    /** Just adds the variances */
    public IsotropicNoiseSE2 plus(IsotropicNoiseSE2 other) {
        return new IsotropicNoiseSE2(
                m_cartesianVariance + other.m_cartesianVariance,
                m_rotationVariance + other.m_rotationVariance);
    }

    public double cartesianVariance() {
        return m_cartesianVariance;
    }

    public double rotationVariance() {
        return m_rotationVariance;
    }

    public IsotropicNoiseSE2 interpolate(IsotropicNoiseSE2 end, double t) {
        return new IsotropicNoiseSE2(
                MathUtil.interpolate(m_cartesianVariance, end.m_cartesianVariance, t),
                MathUtil.interpolate(m_rotationVariance, end.m_rotationVariance, t));
    }

    /** Standard Deviation, for testing only. */
    public double cartesian() {
        return Math.sqrt(m_cartesianVariance);
    }

    /** Standard Deviation, for testing only. */
    public double rotation() {
        return Math.sqrt(m_rotationVariance);
    }

    @Override
    public String toString() {
        return String.format(
                "IsotropicNoiseSE2 [cartesian=%8.5f, rotation=%8.5f]",
                cartesian(), rotation());
    }

}
