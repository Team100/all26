package org.team100.lib.kinematics.six_dof;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.se3.AccelerationSE3;
import org.team100.lib.geometry.se3.AdjointSE3;
import org.team100.lib.geometry.se3.LieSE3;
import org.team100.lib.geometry.se3.VelocitySE3;
import org.team100.lib.geometry.six_dof.SixDofAcceleration;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;
import org.team100.lib.geometry.six_dof.SixDofVelocity;
import org.team100.lib.geometry.six_dof.SphericalWristConfig;
import org.team100.lib.kinematics.Poe;
import org.team100.lib.kinematics.rr.RRKinematics;
import org.team100.lib.kinematics.rrr_so3.SphericalWristKinematics;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.numbers.N6;

/**
 * Six-DOF kinematics using the Modern Robotics approach.
 * 
 * The tool axis is +x at zero config.
 */
public class SixDofKinematicsPoE implements SixDofKinematics {
    private static final boolean DEBUG = false;

    // Joint positions, in global frame, at zero config
    private final Pose3d M1;
    private final Pose3d M2;
    private final Pose3d M3;
    private final Pose3d M4;
    private final Pose3d M5;
    private final Pose3d M6;
    /** The pose usually called "M". */
    private final Pose3d M7;
    // Screw axes, in global frame, at zero config
    private final Twist3d S1;
    private final Twist3d S2;
    private final Twist3d S3;
    private final Twist3d S4;
    private final Twist3d S5;
    private final Twist3d S6;
    /** For solving the positional subproblem */
    private final RRKinematics rrk;
    /** For solving the wrist */
    private final SphericalWristKinematics wk;
    /** Height of the shoulder */
    private final double base;
    /** Tool length from wrist origin, for IK */
    private final double tool;

    public SixDofKinematicsPoE(double base, double boom, double stick, double tool) {
        // base
        M1 = new Pose3d(0, 0, 0, Rotation3d.kZero);
        // shoulder
        M2 = new Pose3d(0, 0, base, Rotation3d.kZero);
        // elbow
        M3 = new Pose3d(boom, 0, base, Rotation3d.kZero);
        // wrist pointing at +x
        M4 = new Pose3d(boom + stick, 0, base, Rotation3d.kZero);
        // wrist pointing at +x
        M5 = new Pose3d(boom + stick, 0, base, Rotation3d.kZero);
        // wrist pointing at +x, this is tool flange
        M6 = new Pose3d(boom + stick, 0, base, Rotation3d.kZero);
        // tool point, is pointing at +x, at full extension
        M7 = new Pose3d(boom + stick + tool, 0, base, Rotation3d.kZero);
        // joint 1 (base) around z
        S1 = Poe.S(VecBuilder.fill(0, 0, 1), new Translation3d(0, 0, 0));
        // joint 2 (shoulder) around -y
        S2 = Poe.S(VecBuilder.fill(0, -1, 0), new Translation3d(0, 0, base));
        // joint 3 (elbow) around -y
        S3 = Poe.S(VecBuilder.fill(0, -1, 0), new Translation3d(boom, 0, base));
        // joint 4 (wrist roll) around +x
        S4 = Poe.S(VecBuilder.fill(1, 0, 0), new Translation3d(boom + stick, 0, base));
        // joint 5 (wrist pitch) around -y
        S5 = Poe.S(VecBuilder.fill(0, -1, 0), new Translation3d(boom + stick, 0, base));
        // joint 6 (tool roll) around +x
        S6 = Poe.S(VecBuilder.fill(1, 0, 0), new Translation3d(boom + stick, 0, base));
        this.base = base;
        this.tool = tool;

        rrk = new RRKinematics(boom, stick);
        wk = new SphericalWristKinematics();
    }

    /** Compose exponentials for each joint pose. */
    @Override
    public SixDofPose forward(SixDofConfig q) {
        Pose3d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose3d eS2q2 = GeometryUtil.exp(S2, q.q2());
        Pose3d eS3q3 = GeometryUtil.exp(S3, q.q3());
        Pose3d eS4q4 = GeometryUtil.exp(S4, q.q4());
        Pose3d eS5q5 = GeometryUtil.exp(S5, q.q5());
        Pose3d eS6q6 = GeometryUtil.exp(S6, q.q6());
        Pose3d p1 = eS1q1;
        Pose3d p2 = GeometryUtil.compose(p1, eS2q2);
        Pose3d p3 = GeometryUtil.compose(p2, eS3q3);
        Pose3d p4 = GeometryUtil.compose(p3, eS4q4);
        Pose3d p5 = GeometryUtil.compose(p4, eS5q5);
        Pose3d p6 = GeometryUtil.compose(p5, eS6q6);
        // return *all* the poses
        return new SixDofPose(
                M1,
                GeometryUtil.compose(p1, M2),
                GeometryUtil.compose(p2, M3),
                GeometryUtil.compose(p3, M4),
                GeometryUtil.compose(p4, M5),
                GeometryUtil.compose(p5, M6),
                GeometryUtil.compose(p6, M7));
    }

    /**
     * Forward velocity kinematics
     * 
     * \dot{x} = J(q) \dot{q}
     * 
     * The Jacobian can be constructed columnwise, where each column
     * is the adjoint map of the exponentials up to that joint, applied
     * to the screw axis of that joint.
     * 
     * See
     * https://publish.illinois.edu/ece470-intro-robotics/files/2024/02/ECE470Lec9-2-1.pdf
     */
    @Override
    public VelocitySE3 forward(SixDofConfig q, SixDofVelocity qdot) {
        Matrix<N6, N6> J = J(q);
        return VelocitySE3.fromVector(J.times(qdot.toVector()));
    }

    @Override
    public AccelerationSE3 forward(SixDofConfig q, SixDofVelocity qdot, SixDofAcceleration qddot) {
        Matrix<N6, N6> J = J(q);
        Matrix<N6, N6> Jdot = Jdot(q, qdot);
        return AccelerationSE3.fromVector(
                Jdot.times(qdot.toVector()).plus(J.times(qddot.toVector())));
    }

    /**
     * Inverse position kinematics: joint configs from cartesian pose.
     *
     * Zero, one, two, four, or eight solutions.
     * 
     * For defaults, use the previous value, or null if you have no idea (and in
     * that case, catch the exception that may occur).
     * 
     * @param p         Tool point pose.
     * @param q1Default In case of base singularity.
     * @param q4Default In case of wrist singularity.
     */
    @Override
    public List<SixDofConfig> inverse(Pose3d p, Double q1Default, Double q4Default) {
        Translation3d t = p.getTranslation();
        if (DEBUG)
            System.out.printf("t %s\n", StrUtil.transStr(t));

        // Wrist rotation is tool rotation.
        Rotation3d R = p.getRotation();

        // Tool translation = tool translation in tool frame, rotated by R.
        Translation3d b = new Translation3d(tool, 0, 0).rotateBy(R);
        // Wrist origin = start at tool point, walk backwards along tool.
        Translation3d w = t.minus(b);
        if (DEBUG)
            System.out.printf("w %s\n", StrUtil.transStr(w));
        Translation2d w2d = w.toTranslation2d();
        // One or two swing options
        List<Double> q1List = getQ1(w2d, q1Default);
        if (DEBUG)
            System.out.printf("swing options %d\n", q1List.size());
        List<SixDofConfig> result = new ArrayList<>();
        for (double q1 : q1List) {
            if (DEBUG)
                System.out.printf("swing %f\n", q1);
            List<RRConfig> rrs = rrConfig(w, q1);
            if (DEBUG)
                System.out.printf("RR options %d\n", rrs.size());
            for (RRConfig rr : rrs) {
                double q2 = rr.q1();
                double q3 = rr.q2();
                if (DEBUG)
                    System.out.printf("q2 %f q3 %f\n ", q2, q3);
                List<SphericalWristConfig> wqs = wristQ(R, wristOrigin(q1, q2, q3), q4Default);
                if (DEBUG)
                    System.out.printf("wrist options %d\n", wqs.size());
                for (SphericalWristConfig wq : wqs) {
                    if (DEBUG)
                        System.out.printf("q4 %f q5 %f q6 %f\n", wq.q4(), wq.q5(), wq.q6());
                    result.add(new SixDofConfig(q1, q2, q3, wq.q4(), wq.q5(), wq.q6()));
                }
            }
        }
        return result;
    }

    @Override
    public SixDofVelocity inverse(SixDofConfig q, VelocitySE3 xdot) {
        Matrix<N6, N6> Jinv = Jinv(q);
        return SixDofVelocity.fromVector(Jinv.times(xdot.toVector()));
    }

    @Override
    public SixDofAcceleration inverse(SixDofConfig q, VelocitySE3 xdot, AccelerationSE3 xddot) {
        Matrix<N6, N6> Jinv = Jinv(q);
        SixDofVelocity qdot = SixDofVelocity.fromVector(Jinv.times(xdot.toVector()));
        Matrix<N6, N6> Jdot = Jdot(q, qdot);
        return SixDofAcceleration.fromVector(
                Jinv.times(
                        xddot.toVector().minus(
                                Jdot.times(Jinv.times(xdot.toVector())))));
    }

    /**
     * End-effector Jacobian.
     * 
     * See eq 8 in Mueller https://arxiv.org/pdf/2506.10686v1
     */
    Matrix<N6, N6> J(SixDofConfig q) {
        // exponential terms
        Pose3d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose3d eS2q2 = GeometryUtil.exp(S2, q.q2());
        Pose3d eS3q3 = GeometryUtil.exp(S3, q.q3());
        Pose3d eS4q4 = GeometryUtil.exp(S4, q.q4());
        Pose3d eS5q5 = GeometryUtil.exp(S5, q.q5());
        Pose3d eS6q6 = GeometryUtil.exp(S6, q.q6());
        // exponential terms, recursively composed
        Pose3d e1 = eS1q1;
        Pose3d e2 = GeometryUtil.compose(e1, eS2q2);
        Pose3d e3 = GeometryUtil.compose(e2, eS3q3);
        Pose3d e4 = GeometryUtil.compose(e3, eS4q4);
        Pose3d e5 = GeometryUtil.compose(e4, eS5q5);
        Pose3d e6 = GeometryUtil.compose(e5, eS6q6);
        Pose3d tcp = GeometryUtil.compose(e6, M7);

        // Space Jacobian
        Matrix<N6, N6> Jv = Jv(q);

        Matrix<N6, N6> t = Poe.t(tcp);

        return t.times(Jv);
    }

    /**
     * Space Jacobian.
     */
    Matrix<N6, N6> Jv(SixDofConfig q) {
        // exponential terms
        Pose3d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose3d eS2q2 = GeometryUtil.exp(S2, q.q2());
        Pose3d eS3q3 = GeometryUtil.exp(S3, q.q3());
        Pose3d eS4q4 = GeometryUtil.exp(S4, q.q4());
        Pose3d eS5q5 = GeometryUtil.exp(S5, q.q5());
        Pose3d eS6q6 = GeometryUtil.exp(S6, q.q6());
        // exponential terms, recursively composed
        Pose3d e1 = eS1q1;
        Pose3d e2 = GeometryUtil.compose(e1, eS2q2);
        Pose3d e3 = GeometryUtil.compose(e2, eS3q3);
        Pose3d e4 = GeometryUtil.compose(e3, eS4q4);
        Pose3d e5 = GeometryUtil.compose(e4, eS5q5);
        Pose3d e6 = GeometryUtil.compose(e5, eS6q6);
        Pose3d tcp = GeometryUtil.compose(e6, M7);

        // first column is just the q1 axis; Mueller calls the columns Si
        Vector<N6> JS1 = GeometryUtil.toVec(S1);
        // second column is the q2 axis transformed by the q1 adjoint
        // see eq 7 in Muller https://arxiv.org/pdf/2506.10686v1
        Vector<N6> JS2 = new Vector<>(AdjointSE3.ad(e1).times(GeometryUtil.toVec(S2)));
        Vector<N6> JS3 = new Vector<>(AdjointSE3.ad(e2).times(GeometryUtil.toVec(S3)));
        Vector<N6> JS4 = new Vector<>(AdjointSE3.ad(e3).times(GeometryUtil.toVec(S4)));
        Vector<N6> JS5 = new Vector<>(AdjointSE3.ad(e4).times(GeometryUtil.toVec(S5)));
        Vector<N6> JS6 = new Vector<>(AdjointSE3.ad(e5).times(GeometryUtil.toVec(S6)));

        // Space Jacobian
        Matrix<N6, N6> Jv = new Matrix<>(Nat.N6(), Nat.N6());
        Jv.setColumn(0, JS1);
        Jv.setColumn(1, JS2);
        Jv.setColumn(2, JS3);
        Jv.setColumn(3, JS4);
        Jv.setColumn(4, JS5);
        Jv.setColumn(5, JS6);
        return Jv;
    }

    /**
     * Time-derivative of the space Jacobian.
     * 
     * See Muller, https://arxiv.org/pdf/2506.10686v1
     */
    Matrix<N6, N6> Jdotv(SixDofConfig q, SixDofVelocity qdot) {
        // exponential terms
        Pose3d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose3d eS2q2 = GeometryUtil.exp(S2, q.q2());
        Pose3d eS3q3 = GeometryUtil.exp(S3, q.q3());
        Pose3d eS4q4 = GeometryUtil.exp(S4, q.q4());
        Pose3d eS5q5 = GeometryUtil.exp(S5, q.q5());
        Pose3d eS6q6 = GeometryUtil.exp(S6, q.q6());
        // exponential terms, recursively composed
        Pose3d e1 = eS1q1;
        Pose3d e2 = GeometryUtil.compose(e1, eS2q2);
        Pose3d e3 = GeometryUtil.compose(e2, eS3q3);
        Pose3d e4 = GeometryUtil.compose(e3, eS4q4);
        Pose3d e5 = GeometryUtil.compose(e4, eS5q5);
        Pose3d e6 = GeometryUtil.compose(e5, eS6q6);
        Pose3d tcp = GeometryUtil.compose(e6, M7);

        // first column is just the q1 axis; Mueller calls the columns Si
        Vector<N6> JS1 = GeometryUtil.toVec(S1);
        // second column is the q2 axis transformed by the q1 adjoint
        // see eq 7 in Muller https://arxiv.org/pdf/2506.10686v1
        Vector<N6> JS2 = new Vector<>(AdjointSE3.ad(e1).times(GeometryUtil.toVec(S2)));
        Vector<N6> JS3 = new Vector<>(AdjointSE3.ad(e2).times(GeometryUtil.toVec(S3)));
        Vector<N6> JS4 = new Vector<>(AdjointSE3.ad(e3).times(GeometryUtil.toVec(S4)));
        Vector<N6> JS5 = new Vector<>(AdjointSE3.ad(e4).times(GeometryUtil.toVec(S5)));
        Vector<N6> JS6 = new Vector<>(AdjointSE3.ad(e5).times(GeometryUtil.toVec(S6)));

        // q1 never moves
        Vector<N6> JdotS1 = GeometryUtil.toVec(new Twist3d());
        Vector<N6> JdotS2 = LieSE3.bracket(JS1, JS2).times(qdot.q1dot());
        Vector<N6> JdotS3 = LieSE3.bracket(JS2, JS3).times(qdot.q1dot());
        Vector<N6> JdotS4 = LieSE3.bracket(JS3, JS4).times(qdot.q1dot());
        Vector<N6> JdotS5 = LieSE3.bracket(JS4, JS5).times(qdot.q1dot());
        Vector<N6> JdotS6 = LieSE3.bracket(JS5, JS6).times(qdot.q1dot());

        Matrix<N6, N6> jdotv = new Matrix<>(Nat.N6(), Nat.N6());
        jdotv.setColumn(0, JdotS1);
        jdotv.setColumn(1, JdotS2);
        jdotv.setColumn(2, JdotS3);
        jdotv.setColumn(3, JdotS4);
        jdotv.setColumn(4, JdotS5);
        jdotv.setColumn(5, JdotS6);
        return jdotv;
    }

    /**
     * Time-derivative of the end-effector Jacobian.
     * TODO: dedupe with jdotv
     */
    Matrix<N6, N6> Jdot(SixDofConfig q, SixDofVelocity qdot) {
        // exponential terms
        Pose3d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose3d eS2q2 = GeometryUtil.exp(S2, q.q2());
        Pose3d eS3q3 = GeometryUtil.exp(S3, q.q3());
        Pose3d eS4q4 = GeometryUtil.exp(S4, q.q4());
        Pose3d eS5q5 = GeometryUtil.exp(S5, q.q5());
        Pose3d eS6q6 = GeometryUtil.exp(S6, q.q6());
        // exponential terms, recursively composed
        Pose3d e1 = eS1q1;
        Pose3d e2 = GeometryUtil.compose(e1, eS2q2);
        Pose3d e3 = GeometryUtil.compose(e2, eS3q3);
        Pose3d e4 = GeometryUtil.compose(e3, eS4q4);
        Pose3d e5 = GeometryUtil.compose(e4, eS5q5);
        Pose3d e6 = GeometryUtil.compose(e5, eS6q6);
        Pose3d tcp = GeometryUtil.compose(e6, M7);
        Matrix<N6, N6> t = Poe.t(tcp);

        // Space Jacobian
        Matrix<N6, N6> Jv = Jv(q);
        Matrix<N6, N6> J = t.times(Jv);
        Matrix<N6, N6> Jdotv = Jdotv(q, qdot);

        VelocitySE3 tcpdot = VelocitySE3.fromVector(J.times(qdot.toVector()));

        Matrix<N6, N6> tdot = Poe.tdot(tcpdot);

        Matrix<N6, N6> jdot = tdot.times(Jv).plus(t.times(Jdotv));
        if (DEBUG)
            System.out.printf("jdot %s\n", StrUtil.matStr(jdot));
        return jdot;
    }

    /**
     * Swing joint. Wrist origin must be in the swing plane. One or two solutions.
     * 
     * In the non-singular case, there are two alternatives here: the "no-flip"
     * case, shoulder near zero, and the "flip" case, with the base pointing the
     * opposite way and the shoulder pointing "back" to the same result.
     * 
     * @param w         Wrist position in the xy plane
     * @param q1Default Used if the position is the origin. A good choice would be
     *                  the previous value of q1. If you have no idea, pass null and
     *                  catch the exception.
     */
    static List<Double> getQ1(Translation2d w, Double q1Default) {
        if (w.getNorm() < 1e-3) {
            if (DEBUG)
                System.out.println("base singularity");
            if (q1Default == null)
                throw new IllegalArgumentException("q1Default is null");
            // in this case we don't do both alternatives, just the one default.
            return List.of(q1Default);
        }
        double radians = w.getAngle().getRadians();
        return List.of(radians, MathUtil.angleModulus(radians + Math.PI));
    }

    /**
     * 0, 1, or 2 solutions
     * 
     * @param w  wrist position
     * @param q1 swing configuration
     */
    private List<RRConfig> rrConfig(Translation3d w, double q1) {
        // Is this the "inline" or the "flip" case?
        Rotation2d rot = w.toTranslation2d().getAngle();
        double signum = 0;
        if (MathUtil.isNear(q1, rot.getRadians(), 1e-3))
            // forward
            signum = 1;
        else
            // backward
            signum = -1;
        // Horizontal distance from base to wrist.
        double x = Math.hypot(w.getX(), w.getY()) * signum;
        // Vertical distance from base to wrist..
        double y = w.getZ() - base;
        // RR sub-problem.
        Translation2d end = new Translation2d(x, y);
        // Find the RR configs
        // TODO: default
        return rrk.inverse(end, null);
    }

    /**
     * Wrist config. One (if singular) or two solutions.
     * 
     * @param R         tool origin rotation
     * @param R04       wrist origin rotation
     * @param q4Default in case of singularity, pass null if you have no idea.
     */
    private List<SphericalWristConfig> wristQ(Rotation3d R, Rotation3d R04, Double q4Default) {
        // The RPR wrist rotation is whatever is left.
        Rotation3d R36 = R.relativeTo(R04);
        List<SphericalWristConfig> wq = wk.inverse(R36, q4Default);
        return wq;
    }

    /** The rotation of the wrist origin */
    private Rotation3d wristOrigin(double q1, double q2, double q3) {
        // Each joint pose up to the wrist.

        Pose3d eS1q1 = GeometryUtil.exp(S1, q1);
        Pose3d eS2q2 = GeometryUtil.exp(S2, q2);
        Pose3d eS3q3 = GeometryUtil.exp(S3, q3);
        Pose3d eS4q4 = GeometryUtil.exp(S4, 0);

        Pose3d p1 = eS1q1;
        Pose3d p2 = GeometryUtil.compose(p1, eS2q2);
        Pose3d p3 = GeometryUtil.compose(p2, eS3q3);
        // Wrist origin.
        Pose3d p4 = GeometryUtil.compose(p3, eS4q4);

        // The rotation for zero wrist roll.
        Rotation3d R04 = p4.getRotation();
        if (DEBUG)
            System.out.printf("R04 %s\n", StrUtil.rotStr(R04));
        return R04;
    }

    /** Jacobian pseudo-inverse. */
    private Matrix<N6, N6> Jinv(SixDofConfig q) {
        Matrix<N6, N6> J = J(q);
        return new Matrix<>(J.getStorage().pseudoInverse());
    }

}
