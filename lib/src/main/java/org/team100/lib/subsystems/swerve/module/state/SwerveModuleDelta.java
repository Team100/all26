package org.team100.lib.subsystems.swerve.module.state;

import java.util.Optional;

import org.team100.lib.subsystems.swerve.kinodynamics.struct.SwerveModuleDeltaStruct;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * For kinematics, the module delta is the dx and dy of each corner, i.e. it's a
 * distance and an angle.
 * 
 * In the real world, consecutive module positions will have different angles,
 * and this class needs to make some assumption about what happened in between.
 * The WPI code assumes that the second angle covers the whole period in
 * between. For awhile, we assumed the angle was smoothly varying between start
 * and end, and computed something like the chord line, but I think that
 * produced inconsistent results.
 * 
 * Actuation uses module "state" i.e. velocity, which is applied to the modules,
 * which are assumed to respond instantly.
 * 
 * At higher levels we constrain steering rate to be achievable, but the
 * maximum rate is quite high, over 10 rad/s, so the change in one 0.02s dt is
 * tenths of radians, not negligible.
 * 
 * But the inverse kinematics produces constant-steering deltas, and so should
 * we here, so we use the ending angle for the whole path.
 */
public class SwerveModuleDelta {
    public static final SwerveModuleDeltaStruct struct = new SwerveModuleDeltaStruct();

    /** Straight line distance from start to end. */
    private final double m_distanceMeters;

    /**
     * Angle of the straight line path. It can be empty, in cases where the angle is
     * indeterminate (e.g. calculating the angle required for zero speed). This is
     * not the *difference* in angle from start to end; it is the angle at the end.
     * 
     * Note this is the robot-relative wrapped angle, i.e. it's just arctan for the
     * delta.
     */
    private final Optional<Rotation2d> m_wrappedAngle;

    /** Zero distance, empty angle. */
    public SwerveModuleDelta() {
        m_distanceMeters = 0;
        m_wrappedAngle = Optional.empty();
    }

    public SwerveModuleDelta(double distanceMeters, Optional<Rotation2d> angle) {
        this.m_distanceMeters = distanceMeters;
        // force the angle value to be wrapped
        this.m_wrappedAngle = angle.map((x) -> new Rotation2d(x.getCos(), x.getSin()));
    }

    /**
     * This is only meaningful when using position as a delta.
     * dx and dy are in meters; if both are very small, the rotation is undefined.
     */
    public SwerveModuleDelta(double dx, double dy) {
        if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) {
            // avoid the garbage rotation.
            this.m_distanceMeters = 0.0;
            this.m_wrappedAngle = Optional.empty();
        } else {
            this.m_distanceMeters = Math.hypot(dx, dy);
            this.m_wrappedAngle = Optional.of(new Rotation2d(dx, dy));
        }
    }

    /**
     * Delta for one module, straight line path using the end angle.
     */
    public static SwerveModuleDelta delta(
            SwerveModulePosition100 start,
            SwerveModulePosition100 end) {
        double deltaM = end.distanceMeters() - start.distanceMeters();
        if (end.unwrappedAngle().isPresent()) {
            return new SwerveModuleDelta(deltaM, end.unwrappedAngle());
        }
        // the angle might be empty, if the encoder has failed
        // (which can seem to happen if the robot is *severely* overrunning).
        return new SwerveModuleDelta(0, Optional.empty());
    }

    public double distanceMeters() {
        return m_distanceMeters;
    }

    public Optional<Rotation2d> wrappedAngle() {
        return m_wrappedAngle;
    }

    public double dx() {
        if (Math.abs(m_distanceMeters) < 1e-6 || m_wrappedAngle.isEmpty()) {
            // wheel is stopped, or angle is invalid so pretend it's stopped.
            return 0;
        }
        return m_distanceMeters * m_wrappedAngle.get().getCos();
    }

    public double dy() {
        if (Math.abs(distanceMeters()) < 1e-6 || wrappedAngle().isEmpty()) {
            // wheel is stopped, or angle is invalid so pretend it's stopped.
            return 0;
        }
        return m_distanceMeters * m_wrappedAngle.get().getSin();
    }

    @Override
    public String toString() {
        return "SwerveModuleDelta [distanceMeters=" + m_distanceMeters + ", angle=" + m_wrappedAngle + "]";
    }

}
