package org.team100.lib.subsystems.swerve.module.state;

import java.util.Objects;
import java.util.Optional;

import org.team100.lib.subsystems.swerve.kinodynamics.struct.SwerveModulePosition100Struct;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * This is a copy of {@link edu.wpi.first.math.kinematics.SwerveModulePosition}
 * but with optional rotation, working around the incorrect behavior of
 * Rotation2d(0, 0).
 * 
 * Represents the state of one swerve module.
 * Uses the "unwrapped" angle; consumers should use MathUtil.angleModulus() if
 * they want the wrapped one.
 */
public class SwerveModulePosition100
        implements Comparable<SwerveModulePosition100>,
        Interpolatable<SwerveModulePosition100>,
        StructSerializable {
    private static final boolean DEBUG = false;
    /** SwerveModulePosition struct for serialization. */
    public static final SwerveModulePosition100Struct struct = new SwerveModulePosition100Struct();

    /** Distance measured by the wheel of the module. */
    private final double m_distanceMeters;

    /**
     * Angle of the module. It can be empty, in cases where the angle is
     * indeterminate (e.g. calculating the angle required for zero speed).
     * 
     * This is "unwrapped": its domain is infinite, not periodic within +/- pi.
     */
    private final Optional<Rotation2d> m_unwrappedAngle;

    /** Zero distance and empty angle. */
    public SwerveModulePosition100() {
        m_distanceMeters = 0;
        m_unwrappedAngle = Optional.empty();
    }

    public SwerveModulePosition100(double distanceMeters, Optional<Rotation2d> unwrappedAngle) {
        m_distanceMeters = distanceMeters;
        m_unwrappedAngle = unwrappedAngle;
    }

    public double distanceMeters() {
        return m_distanceMeters;
    }

    public Optional<Rotation2d> unwrappedAngle() {
        return m_unwrappedAngle;
    }

    /**
     * Tests the *unwrapped* angle for equality, not the wrapped one which is the
     * default Rotation2d behavior.
     */
    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof SwerveModulePosition100 other))
            return false;

        if (Math.abs(other.m_distanceMeters - m_distanceMeters) > 1E-9)
            return false;
        if (m_unwrappedAngle.isEmpty() && other.m_unwrappedAngle.isPresent())
            return false;
        if (m_unwrappedAngle.isPresent() && other.m_unwrappedAngle.isEmpty())
            return false;
        if (m_unwrappedAngle.isEmpty() && other.m_unwrappedAngle.isEmpty())
            return true;
        if (Math.abs(m_unwrappedAngle.get().getRadians() - other.m_unwrappedAngle.get().getRadians()) > 1e-9)
            return false;
        return true;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_distanceMeters, m_unwrappedAngle);
    }

    /**
     * Compares two swerve module positions. One swerve module is "greater" than the
     * other if its
     * distance is higher than the other.
     *
     * @param other The other swerve module.
     * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
     */
    @Override
    public int compareTo(SwerveModulePosition100 other) {
        return Double.compare(m_distanceMeters, other.m_distanceMeters);
    }

    @Override
    public String toString() {
        return String.format(
                "SwerveModulePosition(Distance: %.2f m, Angle: %s)", m_distanceMeters, m_unwrappedAngle);
    }

    /**
     * Returns a copy of this swerve module position.
     *
     * @return A copy.
     */
    public SwerveModulePosition100 copy() {
        return new SwerveModulePosition100(m_distanceMeters, m_unwrappedAngle);
    }

    @Override
    public SwerveModulePosition100 interpolate(SwerveModulePosition100 endValue, double t) {
        double distLerp = MathUtil.interpolate(m_distanceMeters, endValue.m_distanceMeters, t);
        if (m_unwrappedAngle.isEmpty() && endValue.m_unwrappedAngle.isEmpty()) {
            // No angle information at all == no idea where we are, just return zero.
            return new SwerveModulePosition100(0.0, Optional.empty());
        }
        if (m_unwrappedAngle.isEmpty()) {
            // Start is unknown but end is known, so use end.
            Rotation2d angleLerp = endValue.m_unwrappedAngle.get();
            return new SwerveModulePosition100(distLerp, Optional.of(angleLerp));
        }
        if (endValue.m_unwrappedAngle.isEmpty()) {
            // Start is known but end is not, so use start.
            Rotation2d angleLerp = m_unwrappedAngle.get();
            return new SwerveModulePosition100(distLerp, Optional.of(angleLerp));
        }
        // Both start and end are known.
        // This is the old method, interpolating the angle.
        // Rotation2d angleLerp = m_unwrappedAngle.get().interpolate(endValue.m_unwrappedAngle.get(), t);
        // Kinematics assumes that the path from start to end consists of straight
        // lines, using the end angles, so that's what we do here too.
        Rotation2d angleLerp = endValue.m_unwrappedAngle.get();
        return new SwerveModulePosition100(distLerp, Optional.of(angleLerp));
    }

    /**
     * Given the start position, roll in the delta direction by the delta distance.
     * Note: this ignores the start direction, and so can produce strange results if
     * the difference between start and end angles is large.
     */
    public SwerveModulePosition100 plus(SwerveModuleDelta delta) {
        double posM = m_distanceMeters + delta.distanceMeters();
        if (delta.wrappedAngle().isPresent()) {
            if (DEBUG) {
                if (m_unwrappedAngle.isPresent()) {
                    // note wrapping here
                    Rotation2d angleDiff = delta.wrappedAngle().get().minus(m_unwrappedAngle.get());
                    if (Math.abs(angleDiff.getRadians()) > 0.2) {
                        System.out.printf("very fast steering start: %f end: %f\n",
                                m_unwrappedAngle.get().getRadians(),
                                delta.wrappedAngle().get().getRadians());
                    }
                }
            }
            return new SwerveModulePosition100(posM, delta.wrappedAngle());
        }
        // if there's no delta angle, we're not going anywhere.
        if (DEBUG)
            System.out.println("no delta angle");
        return this;
    }

}
