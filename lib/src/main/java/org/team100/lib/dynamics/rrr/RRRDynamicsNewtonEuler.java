package org.team100.lib.dynamics.rrr;

import java.util.List;

import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.util.ModernRobotics;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N4;
import org.wpilib.math.numbers.N6;
import org.wpilib.math.util.Nat;

public class RRRDynamicsNewtonEuler {
    // TODO: make g variable, to account for drive base acceleration.
    final Vector<N3> g;
    final List<Matrix<N4, N4>> Mlist;
    final List<Matrix<N6, N6>> Glist;
    final List<Vector<N6>> Slist;

    /**
     * @param g    gravity vector
     * @param m1   mass of link 1
     * @param m2   mass of link 2
     * @param m3   mass of link 3
     * @param l1   link 1 overall length
     * @param l2   link 2 overall length
     * @param l3   link 3 overall length
     * @param lc1  link 1 center of mass distance
     * @param lc2  link 2 center of mass distance
     * @param lc3  link 3 center of mass distance
     * @param izz1 link 1 inertia
     * @param izz2 link 2 inertia
     * @param izz3 link 3 inertia
     */
    public RRRDynamicsNewtonEuler(
            Vector<N3> g,
            double m1, double m2, double m3,
            double l1, double l2, double l3,
            double lc1, double lc2, double lc3,
            double izz1, double izz2, double izz3) {
        this.g = g;
        // the M list is each link frame relative to the preceding one.
        // M01 is at the origin
        Matrix<N4, N4> M01 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        // M12 is at the end of l1
        Matrix<N4, N4> M12 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, l1, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        // M23 is at the end of l2
        Matrix<N4, N4> M23 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, l2, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        // M34, the tool point, is at the end of l3
        Matrix<N4, N4> M34 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, l3, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        Mlist = List.of(M01, M12, M23, M34);
        // The G list is link inertia matrices in each link frame.
        // Note the link frame is not the center of mass.
        // These are for the center of mass frame.
        Matrix<N6, N6> G1com = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                0, 0, 0, 0, 0, 0, //
                0, 0, 0, 0, 0, 0, //
                0, 0, izz1, 0, 0, 0, //
                0, 0, 0, m1, 0, 0, //
                0, 0, 0, 0, m1, 0, //
                0, 0, 0, 0, 0, m1);
        // Adjoint from the CoM frame to the link frame.
        Matrix<N6, N6> Adg1 = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 1, 0, 0, 0, //
                0, 0, 0, 1, 0, 0, //
                0, 0, lc1, 0, 1, 0, //
                0, -lc1, 0, 0, 0, 1);
        Matrix<N6, N6> G2com = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                0, 0, 0, 0, 0, 0, //
                0, 0, 0, 0, 0, 0, //
                0, 0, izz2, 0, 0, 0, //
                0, 0, 0, m2, 0, 0, //
                0, 0, 0, 0, m2, 0, //
                0, 0, 0, 0, 0, m2);
        // from CoM to link frame
        Matrix<N6, N6> Adg2 = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 1, 0, 0, 0, //
                0, 0, 0, 1, 0, 0, //
                0, 0, lc2, 0, 1, 0, //
                0, -lc2, 0, 0, 0, 1);
        Matrix<N6, N6> G3com = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                0, 0, 0, 0, 0, 0, //
                0, 0, 0, 0, 0, 0, //
                0, 0, izz3, 0, 0, 0, //
                0, 0, 0, m3, 0, 0, //
                0, 0, 0, 0, m3, 0, //
                0, 0, 0, 0, 0, m3);
        // from CoM to link frame
        Matrix<N6, N6> Adg3 = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 1, 0, 0, 0, //
                0, 0, 0, 1, 0, 0, //
                0, 0, lc3, 0, 1, 0, //
                0, -lc3, 0, 0, 0, 1);
        // G list in the link frames applies the Adjoints.
        Glist = List.of(
                Adg1.transpose().times(G1com).times(Adg1),
                Adg2.transpose().times(G2com).times(Adg2),
                Adg3.transpose().times(G3com).times(Adg3));
        // The S list is axis screws, [w, v], in the global frame.
        Vector<N6> S1 = VecBuilder.fill(0, 0, 1, 0, 0, 0);
        Vector<N6> S2 = VecBuilder.fill(0, 0, 1, 0, -l1, 0);
        Vector<N6> S3 = VecBuilder.fill(0, 0, 1, 0, -l1 - l2, 0);
        Slist = List.of(S1, S2, S3);
    }

    public RRREffort effort(
            RRRConfig q,
            RRRVelocity qdot,
            RRRAcceleration qddot) {
        Vector<N3> thetalist = q.toVector();
        Vector<N3> dthetalist = qdot.toVector();
        Vector<N3> ddthetalist = qddot.toVector();
        Vector<N6> Ftip = new Vector<>(Nat.N6());
        return RRREffort.fromVector(ModernRobotics.InverseDynamics(
                Nat.N3(), thetalist, dthetalist, ddthetalist, g, Ftip,
                Mlist, Glist, Slist));
    }

    public RRRAcceleration qddot(
            RRRConfig q,
            RRRVelocity qdot,
            RRREffort effort) {
        Vector<N3> thetalist = q.toVector();
        Vector<N3> dthetalist = qdot.toVector();
        Vector<N3> taulist = effort.toVector();
        Vector<N6> Ftip = new Vector<>(Nat.N6());
        return RRRAcceleration.fromVector(ModernRobotics.ForwardDynamics(
                Nat.N3(), thetalist, dthetalist, taulist, g, Ftip,
                Mlist, Glist, Slist));
    }

}
