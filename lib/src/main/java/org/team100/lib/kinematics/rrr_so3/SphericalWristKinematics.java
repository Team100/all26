package org.team100.lib.kinematics.rrr_so3;

import java.util.List;

import org.team100.lib.geometry.six_dof.SphericalWristConfig;
import org.team100.lib.geometry.six_dof.SphericalWristPose;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

/**
 * The RPR spherical wrist has three joints with intersecting axes.
 * 
 * Now the rotation axes are variable.  Previously they were all z, with variable
 * origins, which was confusing.  So at zero config:
 * 
 * * roll: rotates around x
 * * pitch: rotates around -y
 * * roll: rotates around x again
 */
public class SphericalWristKinematics {
    private static final boolean DEBUG = false;

    /** Forward kinematics is simply composition. */
    public SphericalWristPose forward(SphericalWristConfig q) {
        Pose3d p4 = Pose3d.kZero.plus(r4(q.q4()));
        Pose3d p5 = p4.plus(o5()).plus(r5(q.q5()));
        Pose3d p6 = p5.plus(o6()).plus(r6(q.q6()));
        if (DEBUG) {
            System.out.printf("p4  %s\n", StrUtil.poseStr2(p4));
            System.out.printf("p5  %s\n", StrUtil.poseStr2(p5));
            System.out.printf("p6  %s\n", StrUtil.poseStr2(p6));
        }
        return new SphericalWristPose(p4, p5, p6);
    }

    /**
     * Decomposition of R into roll-pitch-roll (XYX) Euler angles.
     * 
     * One (if singular) or two solutions.
     * 
     * The RPR wrist involves a singularity when q4 and q6 are collinear. In
     * that case, the default q4 value will be used, and this returns a single
     * solutoin.
     * 
     * Otherwise, this returns two solutions.
     *
     * https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/KinematicsSingleBody.pdf
     * https://www.geometrictools.com/Documentation/EulerAngles.pdf
     * 
     * @param R         rotation from wrist origin to tool origin
     * @param q4Default in case of singularity, use this value for q4. A good value
     *                  would be the previous result, so that the q4 joint doesn't
     *                  have to move. Pass null if you have no idea, but catch the
     *                  exception (!).
     */
    public List<SphericalWristConfig> inverse(Rotation3d R, Double q4Default) {
        if (DEBUG)
            System.out.printf("R %s\n", StrUtil.rotStr(R));
        Matrix<N3, N3> r = R.toMatrix();
        if (DEBUG)
            System.out.printf("R %s\n", StrUtil.matStr(r));
        double r11 = r.get(0, 0);
        double r12 = r.get(0, 1);
        double r13 = r.get(0, 2);
        double r21 = r.get(1, 0);
        double r22 = r.get(1, 1);
        // double r23 = r.get(1, 2);
        double r31 = r.get(2, 0);
        double r32 = r.get(2, 1);
        // double r33 = r.get(2, 2);

        // 1e-3 means within about 1.5 degrees of zero.
        if (DEBUG)
            System.out.printf("R11 %f\n", r11);
        if (MathUtil.isNear(1, r11, 1e-3)) {
            if (DEBUG)
                System.out.println("Wrist singularity");
            if (q4Default == null)
                throw new IllegalArgumentException("Wrist singularity with no default");
            double q4 = q4Default;
            double q5 = 0;
            double roll = Math.atan2(r32, r22);
            double q6 = MathUtil.angleModulus(roll - q4);
            return List.of(new SphericalWristConfig(q4, q5, q6));
        }

        double q4 = Math.atan2(r21, -r31);
        // Negative sign here because our convention for the orientation of q5 is
        // opposite the XYX convention.
        double q5 = -1.0 * Math.atan2(Math.sqrt(Math.pow(r21, 2) + Math.pow(r31, 2)), r11);
        double q6 = Math.atan2(r12, r13);

        SphericalWristConfig s1 = new SphericalWristConfig(q4, q5, q6);
        SphericalWristConfig s2 = s1.flip();

        if (DEBUG) {
            System.out.printf("s1 %s\n", s1);
            System.out.printf("s2 %s\n", s2);
        }
        return List.of(s1, s2);
    }

       /** Axis of joint 4 is x */
    private Transform3d r4(double q4) {
        return R(1, 0, 0, q4);
    }

    /** Axis of joint 5 is -y */
    private Transform3d r5(double q5) {
        return R(0, -1, 0, q5);
    }

    /** Axis of joint 6 is x */
    private Transform3d r6(double q6) {
        return R(1, 0, 0, q6);
    }

    /** Origin of joint 5: no offset. */
    private Transform3d o5() {
        return new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0, 0, 0));
    }

    /** Origin of joint 6: no offset. */
    private Transform3d o6() {
        return new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0, 0, 0));
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
