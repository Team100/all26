package org.team100.lib.kinematics;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.geometry.se3.VelocitySE3;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.geometry.Twist3d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N6;
import org.wpilib.math.util.Nat;

/** Util for Product-of-Exponentials. */
public class Poe {
    /**
     * revolute joint screw "S" (Lynch) or "Y" (Muller)
     * 
     * S_i=(e_i, y_i x e_i)
     * 
     * axis, e, is always +z in SE2
     * 
     * @param y position of axis
     */
    public static Twist2d S(Translation2d y) {
        return new Twist2d(y.getY(), -y.getX(), 1);
    }

    /**
     * Screw axis
     * 
     * Lynch/Park say this is -w x q
     * see p142 https://hades.mech.northwestern.edu/images/7/7f/MR.pdf
     * 
     * Mueller says this is q x w (which is the same as above)
     * see p2 https://arxiv.org/pdf/2506.10686v1
     * 
     * @param So S_omega, the axis of rotation in the global frame
     * @param a  any point on the axis
     * @return The screw axis of the joint, in the global frame.
     */
    public static Twist3d S(Vector<N3> So, Translation3d a) {
        Vector<N3> Sv = Vector.cross(GeometryUtil.toVec(a), So);
        return new Twist3d(
                Sv.get(0), Sv.get(1), Sv.get(2),
                So.get(0), So.get(1), So.get(2));
    }

    /**
     * Transform representing tool point translation.
     * 
     * | 1 0 -y |
     * | 0 1 x |
     * | 0 0 1 |
     * 
     * Use this to transform the space Jacobian into the end-effector Jacobian
     * 
     * J = T * Jv
     */
    public static Matrix<N3, N3> t(Pose2d tcp) {
        Matrix<N3, N3> t = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                1, 0, -tcp.getY(), //
                0, 1, tcp.getX(), //
                0, 0, 1);
        return t;
    }

    /**
     * Transform representing tool point velocity.
     * 
     * | 0 0 -y |
     * | 0 0 x |
     * | 0 0 1 |
     * 
     * Use this to transform the time-derivative of the space Jacobian
     * into the time-derivative of the end-effector Jacobian, using the
     * derivative of the expression above, and the product rule:
     * 
     * Jdot = Tdot * Jv + T * Jdotv
     */
    public static Matrix<N3, N3> tdot(VelocitySE2 tcpdot) {
        Matrix<N3, N3> tdot = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, 0, -tcpdot.y(), //
                0, 0, tcpdot.x(), //
                0, 0, 0);
        return tdot;
    }

    /**
     * Transform representing tool point translation.
     * 
     * | 1 0 0 0 z -y |
     * | 0 1 0 -z 0 x |
     * | 0 0 1 y -x 0 |
     * | 0 0 0 1 0 0 |
     * | 0 0 0 0 1 0 |
     * | 0 0 0 0 0 1 |
     * 
     * Use this to transform the space Jacobian into the end-effector Jacobian
     * 
     * J = T * Jv
     */
    public static Matrix<N6, N6> t(Pose3d tcp) {
        Matrix<N6, N6> t = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, tcp.getZ(), -tcp.getY(), //
                0, 1, 0, -tcp.getZ(), 0, tcp.getX(), //
                0, 0, 1, tcp.getY(), -tcp.getX(), 0, //
                0, 0, 0, 1, 0, 0, //
                0, 0, 0, 0, 1, 0, //
                0, 0, 0, 0, 0, 1);
        return t;
    }

    /**
     * Transform representing tool point velocity.
     * 
     * | 0 0 0 0 z -y |
     * | 0 0 0 -z 0 x |
     * | 0 0 0 y -x 0 |
     * | 0 0 0 0 0 0 |
     * | 0 0 0 0 0 0 |
     * | 0 0 0 0 0 0 |
     * 
     * Use this to transform the time-derivative of the space Jacobian
     * into the time-derivative of the end-effector Jacobian, using the
     * derivative of the expression above, and the product rule:
     * 
     * Jdot = Tdot * Jv + T * Jdotv
     */
    public static Matrix<N6, N6> tdot(VelocitySE3 tcpdot) {
        Matrix<N6, N6> tdot = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                0, 0, 0, 0, tcpdot.z(), -tcpdot.y(), //
                0, 0, 0, -tcpdot.z(), 0, tcpdot.x(), //
                0, 0, 0, tcpdot.y(), -tcpdot.x(), 0, //
                0, 0, 0, 0, 0, 0, //
                0, 0, 0, 0, 0, 0, //
                0, 0, 0, 0, 0, 0);
        return tdot;
    }

}
