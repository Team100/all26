package org.team100.lib.geometry.se2;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;

/**
 * Acceleration in SE2 in the robot reference frame,
 * analogous to ChassisSpeeds.
 */
public record ChassisAcceleration(double x, double y, double theta) {
    public static ChassisAcceleration ZERO = new ChassisAcceleration(0, 0, 0);
    /**
     * Acceleration from starting and ending speeds.
     * Correctly includes centrifugal acceleration.
     */
    public static ChassisAcceleration diff(
            ChassisSpeeds v0,
            ChassisSpeeds v1,
            double dtSec) {
        Vector<N3> vv0 = GeometryUtil.toVec(v0);
        Vector<N3> vv1 = GeometryUtil.toVec(v1);
        // accel is v_dot plus adjoint(v)
        // adv_x = - omega v_y
        // adv_y = omega v_x
        Vector<N3> dv = vv1.minus(vv0);
        Vector<N3> vdot = dv.div(dtSec);
        // average speeds during dt
        Vector<N3> v = vv1.plus(vv0).div(2);
        Vector<N3> adv = VecBuilder.fill(
                -1.0 * v.get(2) * v.get(1),
                v.get(2) * v.get(0),
                0);
        Vector<N3> a = vdot.plus(adv);
        return fromVector(a);
    }

    public static ChassisAcceleration fromVector(Vector<N3> v) {
        return new ChassisAcceleration(v.get(0), v.get(1), v.get(2));
    }

    public static ChassisAcceleration fromFieldRelative(
            AccelerationSE2 accel, Rotation2d angle) {
        Translation2d rotated = new Translation2d(accel.x(), accel.y()).rotateBy(angle.unaryMinus());
        return new ChassisAcceleration(rotated.getX(), rotated.getY(), accel.theta());
    }

}
