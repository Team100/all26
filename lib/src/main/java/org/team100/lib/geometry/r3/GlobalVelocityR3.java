package org.team100.lib.geometry.r3;

import org.team100.lib.geometry.se2.VelocitySE2;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Velocity in three dimensions, companion to Translation3d.
 * 
 * This is different from VelocitySE2, which is the companion to Pose2d,
 * i.e. velocity of planar rigid transforms.
 */
public record GlobalVelocityR3(double x, double y, double z) {

    public static GlobalVelocityR3 fromPolar(
            Rotation2d azimuth, Rotation2d elevation, double speed) {
        double vx = speed * azimuth.getCos() * elevation.getCos();
        double vy = speed * azimuth.getSin() * elevation.getCos();
        double vz = speed * elevation.getSin();
        return new GlobalVelocityR3(vx, vy, vz);
    }

    /** Pick up the translation component of v, in the XY plane. */
    public static GlobalVelocityR3 fromSe2(VelocitySE2 v) {
        return new GlobalVelocityR3(v.x(), v.y(), 0);
    }

    public GlobalVelocityR3 plus(GlobalVelocityR3 other) {
        return new GlobalVelocityR3(x + other.x, y + other.y, z + other.z);
    }

    public double normXY() {
        return Math.sqrt(x * x + y * y);
    }

}
