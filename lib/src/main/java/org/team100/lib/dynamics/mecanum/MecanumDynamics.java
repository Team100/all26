package org.team100.lib.dynamics.mecanum;

import static org.team100.lib.geometry.GeometryUtil.det;

import org.team100.lib.dynamics.se2.SE2Dynamics;
import org.team100.lib.dynamics.se2.SE2Effort;
import org.team100.lib.geometry.se2.ChassisAcceleration;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

/**
 * Maps desired acceleration in SE2 (in the robot frame)
 * to linear forces produced at each wheel.
 * 
 * Ignores "slip". TODO: add slip.
 */
public class MecanumDynamics {
    private static final double n = Math.sqrt(2) / 2;
    /** Front-left actuation direction */
    private static final Vector<N2> N_FL = VecBuilder.fill(n, -n);
    /** Front-righ5 actuation direction */
    private static final Vector<N2> N_FR = VecBuilder.fill(n, n);
    /** Rear-left actuation direction */
    private static final Vector<N2> N_RL = VecBuilder.fill(n, n);
    /** Rear-right actuation direction */
    private static final Vector<N2> N_RR = VecBuilder.fill(n, -n);

    /** Robot dynamics, to obtain the whole-robot wrench. */
    private final SE2Dynamics m_dyn;
    /** Inverse dynamics matrix. */
    private final Matrix<N4, N3> m_inv;

    public MecanumDynamics(
            double m, double I,
            Translation2d fl, Translation2d fr,
            Translation2d rl, Translation2d rr) {
        m_dyn = new SE2Dynamics(m, I);
        Vector<N2> R_FL = fl.toVector();
        Vector<N2> R_FR = fr.toVector();
        Vector<N2> R_RL = rl.toVector();
        Vector<N2> r_RR = rr.toVector();
        Matrix<N3, N4> fwd = MatBuilder.fill(Nat.N3(), Nat.N4(),
                N_FL.get(0), N_FR.get(0), N_RL.get(0), N_RR.get(0), //
                N_FL.get(1), N_FR.get(1), N_RL.get(1), N_RR.get(1), //
                det(R_FL, N_FL), det(R_FR, N_FR), det(R_RL, N_RL), det(r_RR, N_RR));
        m_inv = new Matrix<>(fwd.getStorage().pseudoInverse());
    }

    /**
     * Returns the wheel effort in Newtons, which is
     * sqrt(2) times the roller force, to account for
     * the angle of the rollers.
     * 
     * @param a acceleration in robot frame.
     */
    public MecanumEffort effort(ChassisAcceleration a) {
        // Compute rigid body wrench.
        SE2Effort se2Effort = m_dyn.effort(a);
        Vector<N3> w = se2Effort.vector();
        // Find contact forces.
        Vector<N4> f = new Vector<>(m_inv.times(w));
        // Project onto wheel axle
        f = f.times(Math.sqrt(2));
        return MecanumEffort.fromVector(f);
    }

}
