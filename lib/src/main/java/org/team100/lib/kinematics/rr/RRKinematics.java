package org.team100.lib.kinematics.rr;

import java.util.List;

import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPosition;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.util.StrUtil;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Nat;

/**
 * Planar serial RR arm kinematics: two revolute joints and two links.
 * 
 * Implementation is analytic using the law of cosines.
 * 
 * Refer to the diagram:
 * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
 */
public class RRKinematics {
    private static final boolean DEBUG = false;

    /** Proximal link length, meters. */
    private final double l1;
    /** Distal link length, meters. */
    private final double l2;

    /**
     * @param l1 Proximal link length, meters.
     * @param l2 Distal link length, meters.
     */
    public RRKinematics(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
    }

    /**
     * Forward position kinematics: cartesian position from joint configuration.
     * 
     * x = f(q)
     */
    public RRPosition forward(RRConfig q) {
        double x1 = l1 * Math.cos(q.q1());
        double y1 = l1 * Math.sin(q.q1());
        double x2 = x1 + l2 * Math.cos(q.q2() + q.q1());
        double y2 = y1 + l2 * Math.sin(q.q2() + q.q1());
        return new RRPosition(
                new Translation2d(x1, y1),
                new Translation2d(x2, y2));
    }

    /**
     * Forward velocity kinematics: cartesian velocity from joint configuration and
     * velocity.
     * 
     * \dot{x} = J \dot{q}
     */
    public VelocityR2 forward(RRConfig q, RRVelocity qdot) {
        Matrix<N2, N2> J = J(q);
        return VelocityR2.fromVector2(J.times(qdot.toVector()));
    }

    /**
     * Forward acceleration kinematics.
     * 
     * \ddot{x} = \dot{J} \dot{q} + J \ddot{q}
     */
    public AccelerationR2 forward(
            RRConfig q, RRVelocity qdot, RRAcceleration qddot) {
        Matrix<N2, N2> J = J(q);
        Matrix<N2, N2> Jdot = Jdot(q, qdot);
        return AccelerationR2.fromVector(
                Jdot.times(qdot.toVector())
                        .plus(J.times(qddot.toVector())));
    }

    /**
     * Inverse position kinematics: joint configuration from cartesian position.
     * 
     * q = f(x)
     * 
     * Returns 0 (infeasible), 1 (singularity), or 2 (usual case) solutions.
     * 
     * Refer to the diagram, or README.md
     * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
     * 
     * For the default, use the previous value, or null if you have no idea (and in
     * that case, catch the exception that may occur). If l1 and l2 are not the
     * same,
     * the singularity is impossible, so you can safely pass null.
     * 
     * @param x         tool point position
     * @param q1Default in case of singularity
     */
    public List<RRConfig> inverse(Translation2d x, Double q1Default) {
        if (DEBUG)
            System.out.printf("t %s\n", StrUtil.transStr(x));
        // Use law of cosines.
        double r = x.getNorm();
        if (r < 1e-3) {
            // This can only occur if l1 and l2 are (nearly) the same,
            // so use the default, and 180 degrees for the elbow.
            // Note: this configuration is not very useful, maybe don't bother?
            if (DEBUG)
                System.out.println("RR singularity");
            if (q1Default == null)
                throw new IllegalArgumentException("RR singularity with no default");
            return List.of(new RRConfig(q1Default, Math.PI));
        }
        double gamma = Math.atan2(x.getY(), x.getX());
        double c1 = (r * r + l1 * l1 - l2 * l2) / (2 * r * l1);
        double beta = Math.acos(c1);
        double c2 = (l1 * l1 + l2 * l2 - r * r) / (2 * l1 * l2);
        double alpha = Math.acos(c2);

        if (Double.isNaN(alpha) || Double.isNaN(beta) || Double.isNaN(gamma)) {
            if (DEBUG) {
                System.out.println("infeasible");
            }
            return List.of();
        }

        double q1up = MathUtil.angleModulus(gamma + beta);
        double q2up = MathUtil.angleModulus(alpha + Math.PI);

        if (Math.abs(q2up) < 1e-3) {
            if (DEBUG)
                System.out.println("elbow singularity");
            return List.of(new RRConfig(q1up, q2up));
        }

        double q1down = MathUtil.angleModulus(gamma - beta);
        double q2down = -q2up;

        return List.of(
                new RRConfig(q1up, q2up),
                new RRConfig(q1down, q2down));
    }

    /**
     * Inverse velocity kinematics.
     * 
     * \dot{q} = J^{-1} \dot{x}
     * 
     * Depends on the choice of configuration, q.
     */
    public RRVelocity inverse(RRConfig q, VelocityR2 xdot) {
        Matrix<N2, N2> Jinv = Jinv(q);
        RRVelocity v = RRVelocity.fromVector(Jinv.times(xdot.toVector()));
        if (DEBUG)
            System.out.printf("v %s\n", v);
        return v;
    }

    /**
     * Inverse acceleration kinematics.
     * 
     * \ddot{q} = J^{-1}(\ddot{x} - \dot{J} J^{-1} \dot{x})
     * 
     * See doc/README.md equation 9
     * 
     * Depends on the choice of configuration, q.
     */
    public RRAcceleration inverse(RRConfig q, VelocityR2 xdot, AccelerationR2 xddot) {
        Matrix<N2, N2> Jinv = Jinv(q);
        RRVelocity qdot = RRVelocity.fromVector(Jinv.times(xdot.toVector()));
        Matrix<N2, N2> Jdot = Jdot(q, qdot);
        return RRAcceleration.fromVector(
                Jinv.times(
                        xddot.toVector().minus(
                                Jdot.times(Jinv.times(xdot.toVector())))));
    }

    ///////////////////////////////////////////////////////////////////

    /**
     * End-effector Jacobian.
     */
    Matrix<N2, N2> J(RRConfig q) {
        double s1 = Math.sin(q.q1());
        double c1 = Math.cos(q.q1());
        double s12 = Math.sin(q.q1() + q.q2());
        double c12 = Math.cos(q.q1() + q.q2());
        return MatBuilder.fill(Nat.N2(), Nat.N2(),
                -l1 * s1 - l2 * s12, -l2 * s12, //
                l1 * c1 + l2 * c12, l2 * c12);
    }

    /**
     * Time-derivative of the end-effector Jacobian.
     */
    Matrix<N2, N2> Jdot(RRConfig q, RRVelocity qdot) {
        double s1 = Math.sin(q.q1());
        double c1 = Math.cos(q.q1());
        double s12 = Math.sin(q.q1() + q.q2());
        double c12 = Math.cos(q.q1() + q.q2());
        double q1dot = qdot.q1dot();
        double q2dot = qdot.q2dot();
        return MatBuilder.fill(Nat.N2(), Nat.N2(), //
                -l1 * c1 * q1dot - l2 * c12 * (q1dot + q2dot), -l1 * c12 * (q1dot + q2dot), //
                -l1 * s1 * q1dot - l2 * s12 * (q1dot + q2dot), -l2 * s12 * (q1dot + q2dot));
    }

    /**
     * Inverse Jacobian.
     * 
     * When singular, some motion is still possible, so this doesn't return zero,
     * just the pseudoinverse. Note this might not be what you want?
     */
    private Matrix<N2, N2> Jinv(RRConfig q) {
        Matrix<N2, N2> J = J(q);
        if (Math.abs(J.det()) < 1e-3) {
            System.out.printf("WARNING: singularity at config %s\n", q.toString());
        }
        return new Matrix<>(J.getStorage().pseudoInverse());
    }
}