package org.team100.lib.kinematics.rr;

import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPose;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.AdjointSE2;
import org.team100.lib.geometry.se2.LieSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.kinematics.Poe;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

/**
 * Using the PoE method, with SE2 poses, not just
 * R2 translations.
 * 
 * See Mueller, https://arxiv.org/pdf/2506.10686v1
 */
public class RRKinematicsPoE {
    private static final boolean DEBUG = false;
    /** Proximal link length, meters. */
    private final double l1;
    /** Distal link length, meters. */
    private final double l2;

    // Muller calls these A_i
    private final Pose2d M1;
    private final Pose2d M2;
    private final Pose2d M3;

    // Muller calls these Y_i
    private final Twist2d S1;
    private final Twist2d S2;

    public RRKinematicsPoE(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
        S1 = Poe.S(new Translation2d(0, 0));
        if (DEBUG)
            System.out.printf("S1 %s\n", StrUtil.twistStr(S1));
        S2 = Poe.S(new Translation2d(l1, 0));
        if (DEBUG)
            System.out.printf("S2 %s\n", StrUtil.twistStr(S2));
        M1 = new Pose2d(0, 0, Rotation2d.kZero);
        M2 = new Pose2d(l1, 0, Rotation2d.kZero);
        M3 = new Pose2d(l1 + l2, 0, Rotation2d.kZero);
    }

    /**
     * Forward positional kinematics.
     * 
     * Includes all joints, in order to check workspace bounds.
     * 
     * See eq 4 in Muller https://arxiv.org/pdf/2506.10686v1
     * See eq 4.14 in Lynch https://hades.mech.northwestern.edu/images/7/7f/MR.pdf
     */
    public RRPose forward(RRConfig q) {
        // exponential terms
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());

        // exponential terms, recursively composed
        Pose2d e1 = eS1q1;
        Pose2d e2 = GeometryUtil.compose(e1, eS2q2);

        // composed with tool-points for each joint, and the TCP
        Pose2d p1 = M1;
        Pose2d p2 = GeometryUtil.compose(e1, M2);
        Pose2d p3 = GeometryUtil.compose(e2, M3);
        return new RRPose(p1, p2, p3);
    }

    /**
     * Forward velocity kinematics for the end-effector.
     * 
     * \dot{x} = J \dot{q}
     * 
     * See eq 8 in Muller https://arxiv.org/pdf/2506.10686v1
     */
    public VelocitySE2 forward(RRConfig q, RRVelocity qdot) {
        Matrix<N3, N2> J = J(q);
        return VelocitySE2.fromVector(J.times(qdot.toVector()));
    }

    /**
     * Forward acceleration kinematics for the end-effector.
     * 
     * \ddot{x} = \dot{J} \dot{q} + J \ddot{q}
     */
    public AccelerationSE2 forward(
            RRConfig q, RRVelocity qdot, RRAcceleration qddot) {
        Matrix<N3, N2> J = J(q);
        Matrix<N3, N2> Jdot = Jdot(q, qdot);
        return AccelerationSE2.fromVector(
                Jdot.times(qdot.toVector())
                        .plus(J.times(qddot.toVector())));
    }

    ///////////////////////////////////////////
    //
    // Inverses are copied from RRKinematics.

    /**
     * Inverse position kinematics: joint configuration from cartesian position.
     * 
     * q = f(x)
     * 
     * Returns 0 (infeasible), 1 (singularity), or 2 (usual case) solutions.
     * 
     * TODO: supply q1 default in case of the singularity at the origin.
     * 
     * Refer to the diagram, or README.md
     * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
     */
    public List<RRConfig> inverse(Translation2d x) {
        if (DEBUG)
            System.out.printf("t %s\n", StrUtil.transStr(x));
        // Use law of cosines.
        double r = x.getNorm();
        // TODO: handle zero r
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
        Matrix<N2, N2> Jdot = Jdot(q, qdot).block(2, 2, 0, 0);
        return RRAcceleration.fromVector(
                Jinv.times(
                        xddot.toVector().minus(
                                Jdot.times(Jinv.times(xdot.toVector())))));
    }

    ///////////////////////////////////////////////////

    /**
     * End-effector Jacobian.
     * 
     * See eq 8 in Mueller https://arxiv.org/pdf/2506.10686v1
     */
    Matrix<N3, N2> J(RRConfig q) {
        // exponential terms, remember Muller calls Si Yi
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());
        // exponential terms, recursively composed
        Pose2d e1 = eS1q1;
        Pose2d e2 = GeometryUtil.compose(e1, eS2q2);
        Pose2d tcp = GeometryUtil.compose(e2, M3);

        // Space Jacobian
        Matrix<N3, N2> Jv = Jv(q);

        // Tool translation
        Matrix<N3, N3> t = Poe.t(tcp);

        return t.times(Jv);
    }

    /** Spatial Jacobian. */
    Matrix<N3, N2> Jv(RRConfig q) {
        // exponential terms, remember Muller calls Si Yi
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        // Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());
        // exponential terms, recursively composed
        Pose2d e1 = eS1q1;
        // Pose2d e2 = GeometryUtil.compose(e1, eS2q2);
        // Pose2d tcp = GeometryUtil.compose(e2, M3);

        // first column is just the q1 axis; Mueller calls the columns Si
        Vector<N3> JS1 = GeometryUtil.toVec(S1);
        if (DEBUG)
            System.out.printf("JS1 %s\n", StrUtil.vecStr(JS1));

        // second column is the q2 axis transformed by the q1 adjoint
        // see eq 7 in Muller https://arxiv.org/pdf/2506.10686v1
        Vector<N3> JS2 = new Vector<>(AdjointSE2.ad(e1).times(GeometryUtil.toVec(S2)));
        if (DEBUG)
            System.out.printf("JS2 %s\n", StrUtil.vecStr(JS2));

        // Space Jacobian
        Matrix<N3, N2> Jv = new Matrix<>(Nat.N3(), Nat.N2());
        Jv.setColumn(0, JS1);
        Jv.setColumn(1, JS2);
        return Jv;
    }

    /** Time-derivative of space jacobian */
    Matrix<N3, N2> Jdotv(RRConfig q, RRVelocity qdot) {
        // exponential terms
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());
        // exponential terms, recursively composed
        Pose2d e1 = eS1q1;
        Pose2d e2 = GeometryUtil.compose(e1, eS2q2);
        Pose2d tcp = GeometryUtil.compose(e2, M3);

        // first column is just the q1 axis; Mueller calls the columns Si
        Vector<N3> JS1 = GeometryUtil.toVec(S1);
        if (DEBUG)
            System.out.printf("JS1 %s\n", StrUtil.vecStr(JS1));
        // second column is the q2 axis transformed by the q1 adjoint
        // see eq 7 in Muller https://arxiv.org/pdf/2506.10686v1
        Vector<N3> JS2 = new Vector<>(AdjointSE2.ad(e1).times(GeometryUtil.toVec(S2)));
        if (DEBUG)
            System.out.printf("JS2 %s\n", StrUtil.vecStr(JS2));

        // q1 never moves
        Vector<N3> JdotS1 = GeometryUtil.toVec(new Twist2d());
        if (DEBUG)
            System.out.printf("JdotS1 %s\n", StrUtil.vecStr(JdotS1));
        Vector<N3> JdotS2 = LieSE2.bracket(JS1, JS2).times(qdot.q1dot());
        if (DEBUG)
            System.out.printf("JdotS2 %s\n", StrUtil.vecStr(JdotS2));

        Matrix<N3, N2> jdotv = new Matrix<>(Nat.N3(), Nat.N2());
        jdotv.assignBlock(0, 0, JdotS1);
        jdotv.assignBlock(0, 1, JdotS2);
        return jdotv;
    }

    /**
     * Time-derivative of the end-effector Jacobian.
     * 
     * See Muller, https://arxiv.org/pdf/2506.10686v1
     */
    Matrix<N3, N2> Jdot(RRConfig q, RRVelocity qdot) {
        // exponential terms
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());
        // exponential terms, recursively composed
        Pose2d e1 = eS1q1;
        Pose2d e2 = GeometryUtil.compose(e1, eS2q2);
        Pose2d tcp = GeometryUtil.compose(e2, M3);
        Matrix<N3, N3> t = Poe.t(tcp);

        // Space Jacobian
        Matrix<N3, N2> Jv = Jv(q);
        Matrix<N3, N2> J = t.times(Jv);
        Matrix<N3, N2> Jdotv = Jdotv(q, qdot);

        // to convert Jdotv into Jdot, observe that
        // J = T Jv
        // where T is the translation used above.
        // to find Jdot, use the product rule
        // Jdot = Tdot Jv + T Jdotv

        VelocitySE2 tcpdot = VelocitySE2.fromVector(J.times(qdot.toVector()));

        Matrix<N3, N3> tdot = Poe.tdot(tcpdot);

        Matrix<N3, N2> jdot = tdot.times(Jv).plus(t.times(Jdotv));
        if (DEBUG)
            System.out.printf("jdot %s\n", StrUtil.matStr(jdot));
        return jdot;
    }

    /**
     * Jacobian pseudo-inverse, just for cartesian part, since the
     * RR linkage can't handle rotation independently.
     */
    private Matrix<N2, N2> Jinv(RRConfig q) {
        Matrix<N2, N2> J = J(q).block(2, 2, 0, 0);
        return new Matrix<>(J.getStorage().pseudoInverse());
    }
}
