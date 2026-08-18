package org.team100.lib.dynamics.differential;

import static org.team100.lib.geometry.GeometryUtil.det;

import org.team100.lib.dynamics.se2.SE2Dynamics;
import org.team100.lib.dynamics.se2.SE2Effort;
import org.team100.lib.geometry.se2.ChassisAcceleration;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

/**
 * Maps desired "chassis acceleration: (i.e. SE2 in the ROBOT FRAME)
 * to linear forces produced at each wheel. Ignores centrifugal
 * acceleration.
 */
public class DifferentialDriveDynamics {
    /** Left-side actuation direction */
    private static final Vector<N2> N_1 = VecBuilder.fill(1, 0);
    /** Right-side actuation direction */
    private static final Vector<N2> N_2 = VecBuilder.fill(1, 0);

    /** Robot dynamics, to obtain the whole-robot wrench. */
    private final SE2Dynamics m_dyn;
    /** Inverse dynamics matrix. */
    private final Matrix<N2, N3> m_inv;

    public DifferentialDriveDynamics(
            double m, double I, double trackWidthM) {
        m_dyn = new SE2Dynamics(m, I);
        // Left-side wheel location.
        Vector<N2> R_1 = VecBuilder.fill(0, trackWidthM / 2);
        // Right-side wheel location.
        Vector<N2> R_2 = VecBuilder.fill(0, -trackWidthM / 2);
        Matrix<N3, N2> fwd = MatBuilder.fill(Nat.N3(), Nat.N2(),
                N_1.get(0), N_2.get(0), //
                N_1.get(1), N_2.get(1), //
                det(R_1, N_1), det(R_2, N_2));
        m_inv = new Matrix<>(fwd.getStorage().pseudoInverse());
    }

    /**
     * Effort for the supplied acceleration.
     * 
     * IMPORTANT: does not correctly handle lateral acceleration,
     * since it's hard to do anything about it. The correct thing
     * would be to add some extra rotation, to induce a slip-angle
     * on the tires, but that can't be done here.
     * 
     * @param a acceleration in robot frame
     **/
    public DifferentialDriveEffort effort(ChassisAcceleration a) {
        // This is just F=ma and T=Ialpha
        SE2Effort se2Effort = m_dyn.effort(a);
        // This wrench may have a y component which is ignored
        Vector<N3> w = se2Effort.vector();
        Vector<N2> f = new Vector<N2>(m_inv.times(w));
        return DifferentialDriveEffort.fromVector(f);
    }

}
