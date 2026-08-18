package org.team100.lib.geometry.lynx_arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Workspace pose of each joint. The rotation is the rotation of the parent
 * link, in workspace coordinates, not relative to the previous.
 */
public record LynxArmPose(
        Pose3d p1,
        Pose3d p2,
        Pose3d p3,
        Pose3d p4,
        Pose3d p5,
        Pose3d p6) {

    /**
     * Interpolate between a and b by interpolating Euler angles,
     * never crossing pi.
     */
    public static Pose3d interpolate(Pose3d a, Pose3d b, double x) {
        if (x <= 0.0) {
            return a;
        }
        if (x >= 1.0) {
            return b;
        }
        Translation3d aT = a.getTranslation();
        Translation3d bT = b.getTranslation();
        Rotation3d aR = a.getRotation();
        Rotation3d bR = b.getRotation();
        // each translation axis is interpolated separately
        Translation3d lerpT = aT.interpolate(bT, x);
        // Rotation3d lerpR = aR.interpolate(bR, x);
        Rotation3d lerpR = interpolate(aR, bR, x);
        return new Pose3d(lerpT, lerpR);
    }

    /**
     * Interpolate Euler angles, never crossing pi.
     */
    public static Rotation3d interpolate(Rotation3d a, Rotation3d b, double x) {
        if (x <= 0.0) {
            return a;
        }
        if (x >= 1.0) {
            return b;
        }
        return new Rotation3d(
                MathUtil.interpolate(a.getX(), b.getX(), x),
                MathUtil.interpolate(a.getY(), b.getY(), x),
                MathUtil.interpolate(a.getZ(), b.getZ(), x));
    }
}