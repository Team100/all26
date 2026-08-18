package org.team100.lib.geometry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N6;

/**
 * Various distance metrics and norms.
 */
public class Metrics {

    /**
     * The angle between the tag normal and the camera location.
     */
    public static double offAxisAngleRad(Transform3d tagInCameraFrame) {
        Transform3d cameraInTagFrame = tagInCameraFrame.inverse();
        Translation3d cameraTranslation = cameraInTagFrame.getTranslation();
        Translation3d cameraDirection = cameraTranslation.div(cameraTranslation.getNorm());
        // Tag orientation is "into the page" but unit normal direction is "out of the
        // page," so the unit normal is negative in x.
        Translation3d tagUnitNormal = new Translation3d(-1, 0, 0);
        double dot = GeometryUtil.dot(cameraDirection, tagUnitNormal);
        double angle = Math.acos(dot);
        return angle;
    }

    /**
     * The distance between translational components. Ignores rotation entirely.
     * 
     * Always non-negative.
     */
    public static double translationalDistance(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation());
    }

    public static double translationalDistance(Pose3d a, Pose3d b) {
        return a.getTranslation().getDistance(b.getTranslation());
    }

    /**
     * Projection of the shortest distance in SE(2) between two poses into the R2
     * (xy) plane.
     * 
     * It doesn't count the rotational part of the distance per se, just the effect
     * of the rotation on the planar part.
     * 
     * A geodesic is the shortest distance (constant twist) in the SE(2) manifold,
     * which takes a path that looks like a little arc in the x/y plane if there is
     * a non-zero theta component.
     * 
     * This function returns the R2 path length of that little arc.
     * 
     * https://vnav.mit.edu/material/04-05-LieGroups-notes.pdf
     * 
     * For distance that ignores the SE(2) coupling, see DeltaSE2, which treats
     * SE(2) as R2xS1.
     */
    public static double projectedDistance(Pose2d a, Pose2d b) {
        return translationalNorm(a.log(b));
    }

    /**
     * L2 norm of only the translational part of the twist; this is the pathwise
     * length of the little arc.
     */
    public static double translationalNorm(Twist2d a) {
        return Math.hypot(a.dx, a.dy);
    }

    public static double translationalNorm(Twist3d a) {
        return Math.sqrt(a.dx * a.dx + a.dy * a.dy + a.dz * a.dz);
    }

    /** The magnitude of the translational velocity. */
    public static double translationalNorm(ChassisSpeeds a) {
        return Math.hypot(a.vxMetersPerSecond, a.vyMetersPerSecond);
    }

    public static double l1Norm(Vector<?> v) {
        double s = 0;
        for (int i = 0; i < v.getNumRows(); ++i) {
            s += Math.abs(v.get(i, 0));
        }
        return s;
    }

    /////////////////////////////////////////////////////////////////
    ///
    /// DANGER ZONE
    ///
    /// Don't use anything below here unless you really know what you're doing

    /*
     * L2 norm of all components of the twist.
     * 
     * Note that the components don't use the same units, so this norm can be
     * confusing.
     * 
     * Don't use it for anything where you compare it to xy planar distances or
     * velocities.
     */
    public static double l2Norm(Twist2d a) {
        return Math.sqrt(a.dx * a.dx + a.dy * a.dy + a.dtheta * a.dtheta);
    }

    /**
     * L2 norm of all components of the twist.
     * 
     * Note that the components don't use the same units, so this norm can be
     * confusing.
     * 
     * It's useful for optimization, because it captures all the dimensions, and
     * zero really is zero, but don't use it for anything else.
     */
    public static double l2Norm(Twist3d t) {
        Vector<N6> v = VecBuilder.fill(t.dx, t.dy, t.dz, t.rx, t.ry, t.rz);
        return v.norm();
    }

    /**
     * Double-geodesic combines the angular distance with the translational
     * distance, weighting 1 radian equal to 1 meter.
     * 
     * This is not the projected geodesic distance, which is zero for spin-in-place.
     * It's just the L2 norm for all three dimensions.
     * 
     * Note the Chirikjian paper below suggests using mass and inertia for weighting
     * 
     * Don't compare this distance to R2 (xy) planar distances.
     * 
     * @see https://vnav.mit.edu/material/04-05-LieGroups-notes.pdf
     * @see https://rpk.lcsr.jhu.edu/wp-content/uploads/2017/08/Partial-Bi-Invariance-of-SE3-Metrics1.pdf
     */
    public static double doubleGeodesicDistance(Pose2d a, Pose2d b) {
        Translation2d tDiff = a.getTranslation().minus(b.getTranslation());
        double tSqDist = GeometryUtil.dot(tDiff, tDiff);
        double aDiff = a.getRotation().minus(b.getRotation()).getRadians();
        if (GeometryUtil.DEBUG)
            System.out.printf("double geodesic distance t %f a %f\n", tSqDist, aDiff * aDiff);
        return Math.sqrt(aDiff * aDiff + tSqDist);
    }
}
