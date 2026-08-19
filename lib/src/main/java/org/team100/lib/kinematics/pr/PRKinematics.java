package org.team100.lib.kinematics.pr;

import java.util.List;

import org.team100.lib.geometry.pr.PRAcceleration;
import org.team100.lib.geometry.pr.PRConfig;
import org.team100.lib.geometry.pr.PRVelocity;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.Nat;


/**
 * Kinematics for the arm/elevator combination.
 * 
 * This system is represented by two measurements: height of the pivot, and
 * angle of the arm. The angle is measured from vertical. There are physical
 * limits in the real system; these are not represented here.
 * 
 * The cartesian coordinate orientation is with X vertical.
 */
public class PRKinematics {
    /** Rotating arm length, meters */
    private final double l;

    /**
     * @param l Rotating arm length, meters
     */
    public PRKinematics(double l) {
        this.l = l;
    }

    /**
     * Forward position kinematics: cartesian position from joint configuration.
     * 
     * x = f(q)
     */
    public Translation2d forward(PRConfig q) {
        double x = q.q1() + l * Math.cos(q.q2());
        double y = l * Math.sin(q.q2());
        return new Translation2d(x, y);
    }

    /**
     * Forward velocity kinematics: cartesian velocity from joint configuration and
     * velocity.
     * 
     * \dot{x} = J(q) \dot{q}
     */
    public VelocityR2 forward(PRConfig q, PRVelocity qdot) {
        Matrix<N2, N2> J = J(q);
        return VelocityR2.fromVector2(J.times(qdot.toVector()));
    }

    /**
     * Forward acceleration kinematics.
     * 
     * \ddot{x} = \dot{J}\dot{q} + J\ddot{q}
     */
    public AccelerationR2 forward(
            PRConfig q, PRVelocity qdot, PRAcceleration qddot) {
        Matrix<N2, N2> J = J(q);
        Matrix<N2, N2> Jdot = Jdot(q, qdot);
        return AccelerationR2.fromVector(
                Jdot.times(qdot.toVector()).plus(J.times(qddot.toVector())));
    }

    /**
     * Inverse position kinematics: joint configuration from cartesian position.
     * 
     * Returns zero ("outside envelope"), one ("straight out"),
     * or two solutions ("arm pointing up" and "arm pointing down").
     */
    public List<PRConfig> inverse(Translation2d t) {
        double x = t.getX();
        double y = t.getY();
        if (Math.abs(y) > l + 1e-6) {
            // outside envelope, no solution
            return List.of();
        }
        if (Math.abs(y) > l - 1e-6) {
            // straight out, one solution
            return List.of(new PRConfig(x, Math.signum(y) * Math.PI / 2));
        }
        // inside envelope, two solutions.
        double q2Up = Math.asin(y / l);
        double q1Up = x - l * Math.cos(q2Up);
        PRConfig armUp = new PRConfig(q1Up, q2Up);
        double q2Down = Math.PI - q2Up;
        double q1Down = x - l * Math.cos(q2Down);
        PRConfig armDown = new PRConfig(q1Down, q2Down);
        return List.of(armUp, armDown);
    }

    /**
     * Inverse velocity kinematics.
     * 
     * \dot{q} = J^{-1}(x) \dot{x}
     * 
     * Depends on choice of configuration, q.
     */
    public PRVelocity inverse(PRConfig q, VelocityR2 xdot) {
        Matrix<N2, N2> Jinv = Jinv(q);
        return PRVelocity.fromVector(Jinv.times(xdot.toVector()));
    }

    /**
     * Inverse acceleration kinematics.
     * 
     * \ddot{q} = J^{-1}(\ddot{x} - \dot{J}J^{-1}\dot{x})
     * 
     * See doc/README.md equation 9
     * 
     * Depends on choice of configuration, q.
     */
    public PRAcceleration inverse(PRConfig q, VelocityR2 xdot, AccelerationR2 xddot) {
        Matrix<N2, N2> Jinv = Jinv(q);
        PRVelocity qdot = PRVelocity.fromVector(Jinv.times(xdot.toVector()));
        Matrix<N2, N2> Jdot = Jdot(q, qdot);
        return PRAcceleration.fromVector(
                Jinv.times(
                        xddot.toVector().minus(
                                Jdot.times(Jinv.times(xdot.toVector())))));
    }

    //////////////////////////////////////////////////////////////////

    /**
     * End-effector Jacobian.
     */
    private Matrix<N2, N2> J(PRConfig q) {
        double s2 = Math.sin(q.q2());
        double c2 = Math.cos(q.q2());
        return MatBuilder.fill(Nat.N2(), Nat.N2(),
                1, -l * s2, //
                0, l * c2);
    }

    /**
     * Time-derivative of the end-effector Jacobian.
     */
    private Matrix<N2, N2> Jdot(PRConfig q, PRVelocity qdot) {
        double c2 = Math.cos(q.q2());
        double s2 = Math.sin(q.q2());
        double q2dot = qdot.q2dot();
        return MatBuilder.fill(Nat.N2(), Nat.N2(), //
                0, -l * c2 * q2dot, //
                0, -l * s2 * q2dot);
    }

    /**
     * Inverse Jacobian.
     * 
     * When singular, some motion is still possible, so this doesn't return zero,
     * just the pseudoinverse. Note this might not be what you want?
     */
    private Matrix<N2, N2> Jinv(PRConfig q) {
        Matrix<N2, N2> J = J(q);
        if (Math.abs(J.det()) < 1e-3) {
            System.out.printf("WARNING: singularity at config %s\n", q.toString());
        }
        return new Matrix<>(J.getStorage().pseudoInverse());
    }

}
