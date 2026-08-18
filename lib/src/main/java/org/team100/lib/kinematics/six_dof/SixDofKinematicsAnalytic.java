package org.team100.lib.kinematics.six_dof;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.se3.AccelerationSE3;
import org.team100.lib.geometry.se3.VelocitySE3;
import org.team100.lib.geometry.six_dof.SixDofAcceleration;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;
import org.team100.lib.geometry.six_dof.SixDofVelocity;
import org.team100.lib.geometry.six_dof.SphericalWristConfig;
import org.team100.lib.geometry.six_dof.SphericalWristPose;
import org.team100.lib.kinematics.rr.RRKinematics;
import org.team100.lib.kinematics.rrr_so3.SphericalWristKinematics;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

/**
 * Kinematics of six-DOF all-revolute arm with spherical wrist, e.g. PUMA,
 * using links (origins) and joints (rotations).
 * 
 * The joint axis is now variable, so the "zero" joint orientations are all the
 * same. (Previously, the joint axis was always "z", with a variable joint
 * origin, but that was harder to visualize (for me), and different from the
 * natural PoE solution.)
 * 
 * The "zero" rotation, with the tool pointing down +x, results in a TCP pose
 * of (0, 0, 0).
 */
public class SixDofKinematicsAnalytic implements SixDofKinematics {
    private static final boolean DEBUG = false;
    /** Height of the shoulder */
    private final double base;
    /** Boom length between shoulder and elbow */
    private final double boom;
    /** Stick length from elbow to wrist */
    private final double stick;
    /** Tool length from wrist origin. */
    private final double tool;
    /** For solving the positional subproblem */
    private final RRKinematics rrk;
    /** For solving the wrist */
    private final SphericalWristKinematics wk;

    public SixDofKinematicsAnalytic(double base, double boom, double stick, double tool) {
        this.base = base;
        this.boom = boom;
        this.stick = stick;
        this.tool = tool;
        rrk = new RRKinematics(boom, stick);
        wk = new SphericalWristKinematics();
    }

    /**
     * Forward position kinematics: cartesian joint poses from joint configurations.
     * 
     * TODO: this returns joint positions *with* the rotation of the child
     * link, which is probably not what we want.
     */
    @Override
    public SixDofPose forward(SixDofConfig q) {
        Pose3d p1 = Pose3d.kZero.plus(o1()).plus(r1(q.q1()));
        Pose3d p2 = p1.plus(o2()).plus(r2(q.q2()));
        Pose3d p3 = p2.plus(o3()).plus(r3(q.q3()));
        if (DEBUG) {
            System.out.printf("p1  %s\n", StrUtil.poseStr2(p1));
            System.out.printf("p2  %s\n", StrUtil.poseStr2(p2));
            System.out.printf("p3  %s\n", StrUtil.poseStr2(p3));
        }

        // wrist origin
        Pose3d p4o = p3.plus(o4());
        SphericalWristPose wp = wk.forward(new SphericalWristConfig(q.q4(), q.q5(), q.q6()));
        Pose3d p4 = p4o.plus(new Transform3d(Pose3d.kZero, wp.p4()));
        Pose3d p5 = p4o.plus(new Transform3d(Pose3d.kZero, wp.p5()));
        Pose3d p6 = p4o.plus(new Transform3d(Pose3d.kZero, wp.p6()));
        Pose3d tcp = p6.plus(tool());
        if (DEBUG) {
            System.out.printf("p6  %s\n", StrUtil.poseStr2(p6));
            System.out.printf("tcp %s\n", StrUtil.poseStr2(tcp));
        }
        return new SixDofPose(p1, p2, p3, p4, p5, p6, tcp);
    }

    /**
     * Forward velocity kinematics
     * \dot{x} = J(q) \dot{q}
     */
    public VelocitySE3 forward(SixDofConfig q, SixDofVelocity qdot) {
        return null;
        // TODO: finish this
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
     * @param q4Default In case of wrst singularity.
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
    public AccelerationSE3 forward(SixDofConfig q, SixDofVelocity qdot, SixDofAcceleration qddot) {
        // TODO: finish this
        return null;
    }

    @Override
    public SixDofVelocity inverse(SixDofConfig q, VelocitySE3 xdot) {
        // TODO: finish this
        return null;
    }

    @Override
    public SixDofAcceleration inverse(SixDofConfig q, VelocitySE3 xdot, AccelerationSE3 xddot) {
        // TODO: finish this
        return null;
    }

    //////////////////////////////////////////////////////////////////

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
        Pose3d p1 = Pose3d.kZero.plus(o1()).plus(r1(q1));
        Pose3d p2 = p1.plus(o2()).plus(r2(q2));
        Pose3d p3 = p2.plus(o3()).plus(r3(q3));
        // Wrist origin.
        Pose3d p4 = p3.plus(o4());
        // The rotation for zero wrist roll.
        Rotation3d R04 = p4.getRotation();
        if (DEBUG)
            System.out.printf("R04 %s\n", StrUtil.rotStr(R04));
        return R04;
    }

    /** Axis of joint 1 is z */
    private Transform3d r1(double q1) {
        return R(0, 0, 1, q1);
    }

    /** Axis of joint 2 is -y */
    private Transform3d r2(double q2) {
        return R(0, -1, 0, q2);
    }

    /** Axis of joint 3 is -y */
    private Transform3d r3(double q3) {
        return R(0, -1, 0, q3);
    }

    /** Origin of joint 1: no offset. */
    private Transform3d o1() {
        return new Transform3d(
                Translation3d.kZero,
                Rotation3d.kZero);
    }

    /** Origin of joint 2: offset up. */
    private Transform3d o2() {
        return new Transform3d(
                new Translation3d(0, 0, base),
                new Rotation3d(0, 0, 0));
    }

    /** Origin of joint 3: offset out. */
    private Transform3d o3() {
        return new Transform3d(
                new Translation3d(boom, 0, 0),
                Rotation3d.kZero);
    }

    /** Origin of joint 4: offset out. */
    private Transform3d o4() {
        return new Transform3d(
                new Translation3d(stick, 0, 0),
                new Rotation3d(0, 0, 0));
    }

    /** Tool center point. */
    private Transform3d tool() {
        return new Transform3d(
                new Translation3d(tool, 0, 0),
                Rotation3d.kZero);
    }

    /** Rotate in child frame */
    private Transform3d R(double x, double y, double z, double q) {
        return new Transform3d(Translation3d.kZero, new Rotation3d(v(x, y, z), q));
    }

    /** convenience method for vector */
    private Vector<N3> v(double x, double y, double z) {
        return VecBuilder.fill(x, y, z);
    }
}
