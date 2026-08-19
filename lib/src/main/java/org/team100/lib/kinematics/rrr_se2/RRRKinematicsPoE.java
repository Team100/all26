package org.team100.lib.kinematics.rrr_se2;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRPose;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.AdjointSE2;
import org.team100.lib.geometry.se2.LieSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.kinematics.Poe;
import org.team100.lib.kinematics.rr.RRKinematics;
import org.team100.lib.util.StrUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Nat;

/**
 * Planar 3R mechanism in SE2, using Product of Exponentials.
 * 
 * Includes analytic inverse for the wrist position.
 * 
 * Zero position is extended along x.
 */
public class RRRKinematicsPoE {
    private static final boolean DEBUG = false;
    public final double l1;
    public final double l2;
    public final double l3;

    // Joint positions, in global frame, at zero config
    private final Pose2d M1;
    private final Pose2d M2;
    private final Pose2d M3;
    private final Pose2d M4;
    // Screw axes, in global frame, at zero config
    private final Twist2d S1;
    private final Twist2d S2;
    private final Twist2d S3;
    /** For solving the positional subproblem */
    private final RRKinematics rrk;

    /**
     * @param l1 upper arm length
     * @param l2 lower arm length
     * @param l3 tool length
     */
    public RRRKinematicsPoE(double l1, double l2, double l3) {
        this.l1 = l1;
        this.l2 = l2;
        this.l3 = l3;

        // shoulder
        M1 = new Pose2d(0, 0, Rotation2d.kZero);
        // elbow
        M2 = new Pose2d(l1, 0, Rotation2d.kZero);
        // wrist
        M3 = new Pose2d(l1 + l2, 0, Rotation2d.kZero);
        // tool point
        M4 = new Pose2d(l1 + l2 + l3, 0, Rotation2d.kZero);

        // joint 1 (base) around z
        S1 = Poe.S(new Translation2d(0, 0));
        // joint 2 (shoulder) around -y
        S2 = Poe.S(new Translation2d(l1, 0));
        // joint 3 (elbow) around -y
        S3 = Poe.S(new Translation2d(l1 + l2, 0));

        rrk = new RRKinematics(l1, l2);

    }

    /** Compose exponentials for each joint. */
    public RRRPose forward(RRRConfig q) {
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());
        Pose2d eS3q3 = GeometryUtil.exp(S3, q.q3());
        Pose2d p1 = eS1q1;
        Pose2d p2 = GeometryUtil.compose(p1, eS2q2);
        Pose2d p3 = GeometryUtil.compose(p2, eS3q3);
        return new RRRPose(
                M1,
                GeometryUtil.compose(p1, M2),
                GeometryUtil.compose(p2, M3),
                GeometryUtil.compose(p3, M4));
    }

    public VelocitySE2 forward(RRRConfig q, RRRVelocity qdot) {
        Matrix<N3, N3> J = J(q);
        return VelocitySE2.fromVector(J.times(qdot.toVector()));
    }

    public AccelerationSE2 forward(RRRConfig q, RRRVelocity qdot, RRRAcceleration qddot) {
        Matrix<N3, N3> J = J(q);
        Matrix<N3, N3> Jdot = Jdot(q, qdot);
        return AccelerationSE2.fromVector(
                Jdot.times(qdot.toVector()).plus(J.times(qddot.toVector())));
    }

    public List<RRRConfig> inverse(Pose2d x, Double q1Default) {
        Translation2d t = x.getTranslation();
        // Tool rotation.
        Rotation2d R = x.getRotation();
        // Tool translation = tool translation in tool frame, rotated by R.
        Translation2d b = new Translation2d(l3, 0).rotateBy(R);
        // Wrist origin = start at tool point, walk backwards along tool.
        Translation2d w = t.minus(b);
        if (DEBUG)
            System.out.printf("t %s w %s\n", StrUtil.transStr(t), StrUtil.transStr(w));
        List<RRConfig> rrs = rrk.inverse(w, q1Default);
        if (rrs.isEmpty()) {
            System.out.printf("no RR solution for wrist %s\n", StrUtil.transStr(w));
        }
        List<RRRConfig> result = new ArrayList<>();
        for (RRConfig rr : rrs) {
            // Wrist origin rotation
            Rotation2d R03 = new Rotation2d(rr.q1() + rr.q2());
            // The wrist rotation is whatever is left.
            Rotation2d R34 = R.relativeTo(R03);
            result.add(new RRRConfig(rr.q1(), rr.q2(), R34.getRadians()));
        }
        return result;
    }

    public RRRVelocity inverse(RRRConfig q, VelocitySE2 xdot) {
        Matrix<N3, N3> Jinv = Jinv(q);
        return RRRVelocity.fromVector(Jinv.times(xdot.toVector()));
    }

    public RRRAcceleration inverse(RRRConfig q, VelocitySE2 xdot, AccelerationSE2 xddot) {
        Matrix<N3, N3> Jinv = Jinv(q);
        RRRVelocity qdot = RRRVelocity.fromVector(Jinv.times(xdot.toVector()));
        Matrix<N3, N3> Jdot = Jdot(q, qdot);
        return RRRAcceleration.fromVector(
                Jinv.times(
                        xddot.toVector().minus(
                                Jdot.times(Jinv.times(xdot.toVector())))));
    }

    ///////////////////////////////////////////////

    /**
     * End-effector Jacobian
     */
    Matrix<N3, N3> J(RRRConfig q) {
        // exponential terms
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());
        Pose2d eS3q3 = GeometryUtil.exp(S3, q.q3());
        // exponential terms, recursively composed
        Pose2d e1 = eS1q1;
        Pose2d e2 = GeometryUtil.compose(e1, eS2q2);
        Pose2d e3 = GeometryUtil.compose(e2, eS3q3);
        Pose2d tcp = GeometryUtil.compose(e3, M4);
        // Space Jacobian
        Matrix<N3, N3> Jv = Jv(q);
        // Tool translation
        Matrix<N3, N3> t = Poe.t(tcp);
        return t.times(Jv);
    }

    /** Time-derivative of end-effector Jacobian. */
    Matrix<N3, N3> Jdot(RRRConfig q, RRRVelocity qdot) {
        // exponential terms
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());
        Pose2d eS3q3 = GeometryUtil.exp(S3, q.q3());
        // exponential terms, recursively composed
        Pose2d e1 = eS1q1;
        Pose2d e2 = GeometryUtil.compose(e1, eS2q2);
        Pose2d e3 = GeometryUtil.compose(e2, eS3q3);
        Pose2d tcp = GeometryUtil.compose(e3, M4);
        // Tool translation
        Matrix<N3, N3> t = Poe.t(tcp);
        // Space Jacobian
        Matrix<N3, N3> Jv = Jv(q);
        Matrix<N3, N3> J = t.times(Jv);
        Matrix<N3, N3> Jdotv = Jdotv(q, qdot);

        VelocitySE2 tcpdot = VelocitySE2.fromVector(J.times(qdot.toVector()));
        Matrix<N3, N3> tdot = Poe.tdot(tcpdot);
        Matrix<N3, N3> jdot = tdot.times(Jv).plus(t.times(Jdotv));
        return jdot;
    }

    /**
     * Space Jacobian
     */
    Matrix<N3, N3> Jv(RRRConfig q) {
        // exponential terms
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());
        // Pose2d eS3q3 = GeometryUtil.exp(S3, q.q3());
        // exponential terms, recursively composed
        Pose2d e1 = eS1q1;
        Pose2d e2 = GeometryUtil.compose(e1, eS2q2);
        // Pose2d e3 = GeometryUtil.compose(e2, eS3q3);
        // first column is just the q1 axis; Mueller calls the columns Si
        Vector<N3> JS1 = GeometryUtil.toVec(S1);
        // second column is the q2 axis transformed by the q1 adjoint
        // see eq 7 in Muller https://arxiv.org/pdf/2506.10686v1
        Vector<N3> JS2 = new Vector<>(AdjointSE2.ad(e1).times(GeometryUtil.toVec(S2)));
        Vector<N3> JS3 = new Vector<>(AdjointSE2.ad(e2).times(GeometryUtil.toVec(S3)));
        // Space Jacobian
        Matrix<N3, N3> Jv = new Matrix<>(Nat.N3(), Nat.N3());
        Jv.setColumn(0, JS1);
        Jv.setColumn(1, JS2);
        Jv.setColumn(2, JS3);
        return Jv;
    }

    /** Time-derivative of space Jacobian */
    Matrix<N3, N3> Jdotv(RRRConfig q, RRRVelocity qdot) {
        // exponential terms
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());
        Pose2d eS3q3 = GeometryUtil.exp(S3, q.q3());
        // exponential terms, recursively composed
        Pose2d e1 = eS1q1;
        Pose2d e2 = GeometryUtil.compose(e1, eS2q2);
        Pose2d e3 = GeometryUtil.compose(e2, eS3q3);
        Pose2d tcp = GeometryUtil.compose(e3, M4);
        // first column is just the q1 axis; Mueller calls the columns Si
        Vector<N3> JS1 = GeometryUtil.toVec(S1);
        // second column is the q2 axis transformed by the q1 adjoint
        // see eq 7 in Muller https://arxiv.org/pdf/2506.10686v1
        Vector<N3> JS2 = new Vector<>(AdjointSE2.ad(e1).times(GeometryUtil.toVec(S2)));
        Vector<N3> JS3 = new Vector<>(AdjointSE2.ad(e2).times(GeometryUtil.toVec(S3)));
        // q1 never moves
        Vector<N3> JdotS1 = GeometryUtil.toVec(new Twist2d());
        Vector<N3> JdotS2 = LieSE2.bracket(JS1, JS2).times(qdot.q1dot());
        Vector<N3> JdotS3 = LieSE2.bracket(JS2, JS3).times(qdot.q1dot());
        Matrix<N3, N3> jdotv = new Matrix<>(Nat.N3(), Nat.N3());
        jdotv.setColumn(0, JdotS1);
        jdotv.setColumn(1, JdotS2);
        jdotv.setColumn(2, JdotS3);
        return jdotv;

    }

    /**
     * Inverse end-effector Jacobian.
     *
     * When singular, some motion is still possible, so this doesn't return zero,
     * just the pseudoinverse.
     */
    Matrix<N3, N3> Jinv(RRRConfig q) {
        Matrix<N3, N3> J = J(q);
        return new Matrix<>(J.getStorage().pseudoInverse());
    }
}
