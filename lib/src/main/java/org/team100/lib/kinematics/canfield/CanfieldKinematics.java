package org.team100.lib.kinematics.canfield;

import org.team100.lib.geometry.canfield.CanfieldConfig;
import org.team100.lib.util.StrUtil;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Nat;

/**
 * Canfield joint kinematics. See README.md
 */
public class CanfieldKinematics {
    private static final boolean DEBUG = false;

    private final double l;

    private final Vector<N3> qhat1;
    private final Vector<N3> qhat2;
    private final Vector<N3> qhat3;
    private final Vector<N3> b1;
    private final Vector<N3> b2;
    private final Vector<N3> b3;
    private final Vector<N3> u1;
    private final Vector<N3> u2;
    private final Vector<N3> u3;

    /**
     * @param b distance from basal center to joints
     * @param l length of each link
     */
    public CanfieldKinematics(double b, double l) {
        this.l = l;
        // b points are an equilateral triangle centered at the origin
        b1 = VecBuilder.fill(b, 0, 0);
        b2 = VecBuilder.fill(b * Math.cos(2 * Math.PI / 3), b * Math.sin(2 * Math.PI / 3), 0);
        b3 = VecBuilder.fill(b * Math.cos(4 * Math.PI / 3), b * Math.sin(4 * Math.PI / 3), 0);
        qhat1 = b1.times(-1).unit();
        qhat2 = b2.times(-1).unit();
        qhat3 = b3.times(-1).unit();
        Matrix<N3, N3> Rz = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, -1, 0, //
                1, 0, 0, //
                0, 0, 1);
        u1 = new Vector<>(Rz.times(qhat1).times(-1));
        u2 = new Vector<>(Rz.times(qhat2).times(-1));
        u3 = new Vector<>(Rz.times(qhat3).times(-1));
        if (DEBUG) {
            System.out.printf("u %s %s %s\n", u1, u2, u3);
        }
    }

    public Pose3d forward(CanfieldConfig q) {
        Rotation3d R1 = new Rotation3d(u1, q.q1());
        Rotation3d R2 = new Rotation3d(u2, q.q2());
        Rotation3d R3 = new Rotation3d(u3, q.q3());
        Vector<N3> m1 = new Vector<>(R1.toMatrix().times(qhat1.times(l)).plus(b1));
        Vector<N3> m2 = new Vector<>(R2.toMatrix().times(qhat2.times(l)).plus(b2));
        Vector<N3> m3 = new Vector<>(R3.toMatrix().times(qhat3.times(l)).plus(b3));
        Vector<N3> Nhatm = Vector.cross(m2.minus(m1), m3.minus(m2)).unit();
        double del1 = Math.abs(Nhatm.dot(m1.minus(b1)));
        double del2 = Math.abs(Nhatm.dot(m2.minus(b2)));
        double del3 = Math.abs(Nhatm.dot(m3.minus(b3)));
        Vector<N3> d1 = b1.plus(Nhatm.times(2 * del1));
        Vector<N3> d2 = b2.plus(Nhatm.times(2 * del2));
        Vector<N3> d3 = b3.plus(Nhatm.times(2 * del3));
        Vector<N3> cD = d1.plus(d2).plus(d3).div(3);
        Vector<N3> zhatD = Vector.cross(d2.minus(d1), d3.minus(d2)).unit();
        Vector<N3> xhatD = d1.minus(cD).unit();
        Vector<N3> yhatD = Vector.cross(xhatD, zhatD).times(-1);
        Matrix<N3, N3> Rbd = new Matrix<>(Nat.N3(), Nat.N3());
        Rbd.assignBlock(0, 0, xhatD);
        Rbd.assignBlock(0, 1, yhatD);
        Rbd.assignBlock(0, 2, zhatD);
        if (DEBUG) {
            System.out.printf("R %s %s %s\n", StrUtil.rotStr(R1), StrUtil.rotStr(R2), StrUtil.rotStr(R3));
            System.out.printf("m %s %s %s\n", m1, m2, m3);
            System.out.printf("Nhatm %s \n", Nhatm);
            System.out.printf("del %f %f %f\n", del1, del2, del3);
            System.out.printf("d %s %s %s\n", d1, d2, d3);
            System.out.printf("cD %s\n", cD);
            System.out.printf("xyzD %s %s %s\n", xhatD, yhatD, zhatD);
            System.out.printf("Rbd %s\n", Rbd);
        }
        return new Pose3d(
                new Translation3d(cD),
                new Rotation3d(Rbd));
    }

    public void inverse() {
    }
}
