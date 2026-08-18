package org.team100.lib.dynamics.rr;

import java.util.List;

import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.util.ModernRobotics;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;

/**
 * Use Modern Robotics Newton Euler method for the RR case.
 * 
 * The RR case is easy to solve analytically, so this exists to check
 * the Newton Euler code correctness.
 */
public class RRDynamicsNewtonEuler implements RRDynamics {
    // TODO: make g variable, to account for drive base acceleration.
    final Vector<N3> g;
    final List<Matrix<N4, N4>> Mlist;
    final List<Matrix<N6, N6>> Glist;
    final List<Vector<N6>> Slist;

    /**
     * @param m1   mass of link 1
     * @param m2   mass of link 2
     * @param l1   link 1 overall length
     * @param l2   link 2 overall length
     * @param lc1  link 1 center of mass distance
     * @param lc2  link 2 center of mass distance
     * @param izz1 link 1 inertia
     * @param izz2 link 2 inertia
     */
    public RRDynamicsNewtonEuler(
            double m1, double m2,
            double l1, double l2,
            double lc1, double lc2,
            double izz1, double izz2) {
        // gravity points towards -x
        g = VecBuilder.fill(-9.8, 0, 0);
        // The M matrices describe the frames of each link, and of the
        // tool.
        // Each M is the link frame relative to the preceding one.
        // The frames here are at the pivots, so M01 is identity.
        Matrix<N4, N4> M01 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        // M12 is at the second pivot
        Matrix<N4, N4> M12 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, l1, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        // M23 is the tool point at the end of the second link.
        Matrix<N4, N4> M23 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, l2, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        Mlist = List.of(M01, M12, M23);
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
        Matrix<N6, N6> Adg2 = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                1, 0, 0, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 1, 0, 0, 0, //
                0, 0, 0, 1, 0, 0, //
                0, 0, lc2, 0, 1, 0, //
                0, -lc2, 0, 0, 0, 1);
        // G list in the link frames applies the Adjoints.
        Glist = List.of(
                Adg1.transpose().times(G1com).times(Adg1),
                Adg2.transpose().times(G2com).times(Adg2));
        // The S list is axis screws, [w, v], in the global frame.
        Vector<N6> S1 = VecBuilder.fill(0, 0, 1, 0, 0, 0);
        Vector<N6> S2 = VecBuilder.fill(0, 0, 1, 0, -l1, 0);
        Slist = List.of(S1, S2);
    }

    /** Compute effort (torque) for each joint, with zero tip force. */
    @Override
    public RREffort effort(
            RRConfig q,
            RRVelocity qdot,
            RRAcceleration qddot) {
        Vector<N2> thetalist = q.toVector();
        Vector<N2> dthetalist = qdot.toVector();
        Vector<N2> ddthetalist = qddot.toVector();
        Vector<N6> Ftip = new Vector<>(Nat.N6());
        return RREffort.fromVector(ModernRobotics.InverseDynamics(
                Nat.N2(), thetalist, dthetalist, ddthetalist, g, Ftip,
                Mlist, Glist, Slist));
    }

    /** Compute acceleration for each joint. */
    @Override
    public RRAcceleration qddot(
            RRConfig q,
            RRVelocity qdot,
            RREffort effort) {
        Vector<N2> thetalist = q.toVector();
        Vector<N2> dthetalist = qdot.toVector();
        Vector<N2> taulist = effort.toVector();
        Vector<N6> Ftip = new Vector<>(Nat.N6());
        return RRAcceleration.fromVector(ModernRobotics.ForwardDynamics(
                Nat.N2(), thetalist, dthetalist, taulist, g, Ftip,
                Mlist, Glist, Slist));
    }

}
