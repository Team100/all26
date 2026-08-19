package org.team100.lib.dynamics.r;

import java.util.List;

import org.team100.lib.util.ModernRobotics;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N4;
import org.wpilib.math.numbers.N6;
import org.wpilib.math.util.Nat;

/**
 * Use Modern Robotics Newton Euler method for the R case.
 * 
 * The R case is easy to solve analytically, so this exists to check
 * the Newton Euler code correctness.
 */
public class RDynamicsNewtonEuler implements RDynamics {
    // TODO: make g variable, to account for drive base acceleration.
    final Vector<N3> g;
    final List<Matrix<N4, N4>> Mlist;
    final List<Matrix<N6, N6>> Glist;
    final List<Vector<N6>> Slist;

    /**
     * @param m1   mass of link 1
     * @param l1   link 1 overall length
     * @param lc1  link 1 center of mass distance
     * @param izz1 link 1 inertia
     */
    public RDynamicsNewtonEuler(
            double m1,
            double l1,
            double lc1,
            double izz1) {
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
        // M12 is at the tool point
        Matrix<N4, N4> M12 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, l1, //
                0, 1, 0, 0, //
                0, 0, 1, 0, //
                0, 0, 0, 1);
        Mlist = List.of(M01, M12);
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
        // G list in the link frames applies the Adjoints.
        Glist = List.of(
                Adg1.transpose().times(G1com).times(Adg1));
        // The S list is axis screws, [w, v], in the global frame.
        Vector<N6> S1 = VecBuilder.fill(0, 0, 1, 0, 0, 0);
        Slist = List.of(S1);
    }

    /** Compute effort (torque) for each joint, with zero tip force. */
    @Override
    public REffort effort(
            RConfig q,
            RVelocity qdot,
            RAcceleration qddot) {
        Vector<N1> thetalist = q.toVector();
        Vector<N1> dthetalist = qdot.toVector();
        Vector<N1> ddthetalist = qddot.toVector();
        Vector<N6> Ftip = new Vector<>(Nat.N6());
        return REffort.fromVector(ModernRobotics.InverseDynamics(
                Nat.N1(), thetalist, dthetalist, ddthetalist, g, Ftip,
                Mlist, Glist, Slist));
    }

    /** Compute acceleration for each joint. */
    @Override
    public RAcceleration qddot(
            RConfig q,
            RVelocity qdot,
            REffort effort) {
        Vector<N1> thetalist = q.toVector();
        Vector<N1> dthetalist = qdot.toVector();
        Vector<N1> taulist = effort.toVector();
        Vector<N6> Ftip = new Vector<>(Nat.N6());
        return RAcceleration.fromVector(ModernRobotics.ForwardDynamics(
                Nat.N1(), thetalist, dthetalist, taulist, g, Ftip,
                Mlist, Glist, Slist));
    }

}
