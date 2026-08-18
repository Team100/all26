package org.team100.lib.dynamics.six_dof;

import java.util.List;

import org.team100.lib.geometry.six_dof.SixDofAcceleration;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofVelocity;
import org.team100.lib.util.ModernRobotics;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;

/**
 * Six-DOF in the usual arrangement, with spherical wrist.
 * "Home" position is extended down +x.
 */
public class SixDofDynamicsNewtonEuler {
    // TODO: make g variable, to account for drive base acceleration.
    final Vector<N3> g;
    List<Matrix<N4, N4>> Mlist;
    List<Matrix<N6, N6>> Glist;
    List<Vector<N6>> Slist;

    /**
     * Model inertia for l1 l2 l3 as thin rods, no mass for wrist.
     * 
     * @param base  length
     * @param boom  length
     * @param stick length
     * @param tool  length
     * @param m1    base mass
     * @param m2    boom mass
     * @param m3    stick mass
     * @param m4    tool mass
     */
    public SixDofDynamicsNewtonEuler(
            double base, double boom, double stick, double tool,
            double m1, double m2, double m3, double m4) {
        // gravity points towards -z
        g = VecBuilder.fill(0, 0, -9.8);
        // The M matrices describe the frames of each link, and of the
        // tool.
        // Each Mij is the link frame, i, *relative to the preceding one, j*.
        // The frames here are at the pivots
        // base
        Matrix<N4, N4> M01 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        // shoulder
        Matrix<N4, N4> M12 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, base, //
                0, 0, 0, 1);
        // elbow
        Matrix<N4, N4> M23 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, boom, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        // wrist
        Matrix<N4, N4> M34 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, stick, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        // wrist
        Matrix<N4, N4> M45 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        // wrist
        Matrix<N4, N4> M56 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        // wrist
        Matrix<N4, N4> M67 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, tool, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        Mlist = List.of(M01, M12, M23, M34, M45, M56, M67);
        // The G list is link inertia matrices in each link frame.
        // Note the link frame is not the center of mass.
        // These are for the center of mass frame.
        // base inertia is only relevant about z
        // NOTE! these G matrices must have full rank; they represent
        // real inertias, so the diagonals cannot be zero.
        Matrix<N6, N6> G1com = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                0.01, 0, 0, 0, 0, 0, //
                0, 0.01, 0, 0, 0, 0, //
                0, 0, m1 * base * base / 12, 0, 0, 0, //
                0, 0, 0, m1, 0, 0, //
                0, 0, 0, 0, m1, 0, //
                0, 0, 0, 0, 0, m1);
        // Adjoint from the CoM frame to the link frame.
        Matrix<N6, N6> Adg1 = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 1, 0, 0, 0, //
                0, base / 2, 0, 1, 0, 0, //
                -base / 2, 0, 0, 0, 1, 0, //
                0, 0, 0, 0, 0, 1);
        // boom extends along x, so has nonzero inertia about y and z.
        double i2 = m2 * boom * boom / 12;
        Matrix<N6, N6> G2com = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                0.01, 0, 0, 0, 0, 0, //
                0, i2, 0, 0, 0, 0, //
                0, 0, i2, 0, 0, 0, //
                0, 0, 0, m2, 0, 0, //
                0, 0, 0, 0, m2, 0, //
                0, 0, 0, 0, 0, m2);
        Matrix<N6, N6> Adg2 = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 1, 0, 0, 0, //
                0, 0, 0, 1, 0, 0, //
                0, 0, boom / 2, 0, 1, 0, //
                0, -boom / 2, 0, 0, 0, 1);
        // stick extends along x, so has nonzero inertia about y and z.
        double i3 = m3 * stick * stick / 12;
        Matrix<N6, N6> G3com = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                0.01, 0, 0, 0, 0, 0, //
                0, i3, 0, 0, 0, 0, //
                0, 0, i3, 0, 0, 0, //
                0, 0, 0, m3, 0, 0, //
                0, 0, 0, 0, m3, 0, //
                0, 0, 0, 0, 0, m3);
        Matrix<N6, N6> Adg3 = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 1, 0, 0, 0, //
                0, 0, 0, 1, 0, 0, //
                0, 0, stick / 2, 0, 1, 0, //
                0, -stick / 2, 0, 0, 0, 1);
        // wrist roll inertia is about x
        Matrix<N6, N6> G4com = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                0.1, 0, 0, 0, 0, 0, //
                0, 0.01, 0, 0, 0, 0, //
                0, 0, 0.01, 0, 0, 0, //
                0, 0, 0, 0.1, 0, 0, //
                0, 0, 0, 0, 0.1, 0, //
                0, 0, 0, 0, 0, 0.1);
        Matrix<N6, N6> Adg4 = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 1, 0, 0, 0, //
                0, 0, 0, 1, 0, 0, //
                0, 0, 0, 0, 1, 0, //
                0, 0, 0, 0, 0, 1);
        // pitch inertia is about y
        Matrix<N6, N6> G5com = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                0.01, 0, 0, 0, 0, 0, //
                0, 0.1, 0, 0, 0, 0, //
                0, 0, 0.01, 0, 0, 0, //
                0, 0, 0, 0.1, 0, 0, //
                0, 0, 0, 0, 0.1, 0, //
                0, 0, 0, 0, 0, 0.1);
        Matrix<N6, N6> Adg5 = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 1, 0, 0, 0, //
                0, 0, 0, 1, 0, 0, //
                0, 0, 0, 0, 1, 0, //
                0, 0, 0, 0, 0, 1);
        // tool frame includes inertia of the tool itself
        double i4 = m4 * tool * tool / 12;
        Matrix<N6, N6> G6com = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                0.01, 0, 0, 0, 0, 0, //
                0, i4, 0, 0, 0, 0, //
                0, 0, i4, 0, 0, 0, //
                0, 0, 0, m4, 0, 0, //
                0, 0, 0, 0, m4, 0, //
                0, 0, 0, 0, 0, m4);
        Matrix<N6, N6> Adg6 = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 1, 0, 0, 0, //
                0, 0, 0, 1, 0, 0, //
                0, 0, tool / 2, 0, 1, 0, //
                0, -tool / 2, 0, 0, 0, 1);
        // G list in the link frames applies the Adjoints.
        Glist = List.of(
                Adg1.transpose().times(G1com).times(Adg1),
                Adg2.transpose().times(G2com).times(Adg2),
                Adg3.transpose().times(G3com).times(Adg3),
                Adg4.transpose().times(G4com).times(Adg4),
                Adg5.transpose().times(G5com).times(Adg5),
                Adg6.transpose().times(G6com).times(Adg6));
        // The S list is axis screws, [w, v], in the global frame.
        Vector<N6> S1 = VecBuilder.fill(0, 0, 1, 0, 0, 0);
        Vector<N6> S2 = VecBuilder.fill(0, -1, 0, base, 0, 0);
        Vector<N6> S3 = VecBuilder.fill(0, -1, 0, base, 0, -boom);
        Vector<N6> S4 = VecBuilder.fill(1, 0, 0, 0, base, 0);
        Vector<N6> S5 = VecBuilder.fill(0, -1, 0, base, 0, -boom - stick);
        Vector<N6> S6 = VecBuilder.fill(1, 0, 0, 0, base, 0);
        Slist = List.of(S1, S2, S3, S4, S5, S6);
        // Try the zero cases to make sure the problem is well stated

        effort(
                new SixDofConfig(0, 0, 0, 0, 0, 0),
                new SixDofVelocity(0, 0, 0, 0, 0, 0),
                new SixDofAcceleration(0, 0, 0, 0, 0, 0));
        qddot(
                new SixDofConfig(0, 0, 0, 0, 0, 0),
                new SixDofVelocity(0, 0, 0, 0, 0, 0),
                new SixDofEffort(0, 0, 0, 0, 0, 0));
    }

    public SixDofEffort effort(
            SixDofConfig q,
            SixDofVelocity qdot,
            SixDofAcceleration qddot) {
        Vector<N6> thetalist = q.toVector();
        Vector<N6> dthetalist = qdot.toVector();
        Vector<N6> ddthetalist = qddot.toVector();
        Vector<N6> Ftip = new Vector<>(Nat.N6());
        return SixDofEffort.fromVector(ModernRobotics.InverseDynamics(
                Nat.N6(), thetalist, dthetalist, ddthetalist, g, Ftip,
                Mlist, Glist, Slist));
    }

    public SixDofAcceleration qddot(
            SixDofConfig q,
            SixDofVelocity qdot,
            SixDofEffort effort) {
        Vector<N6> thetalist = q.toVector();
        Vector<N6> dthetalist = qdot.toVector();
        Vector<N6> taulist = effort.toVector();
        Vector<N6> Ftip = new Vector<>(Nat.N6());
        return SixDofAcceleration.fromVector(ModernRobotics.ForwardDynamics(
                Nat.N6(), thetalist, dthetalist, taulist, g, Ftip,
                Mlist, Glist, Slist));
    }
}
