package org.team100.lib.dynamics.swerve;

import java.util.Optional;

import org.team100.lib.dynamics.se2.SE2Dynamics;
import org.team100.lib.dynamics.se2.SE2Effort;
import org.team100.lib.dynamics.swerve.SwerveEffort.ModuleEffort;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleState100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

/**
 * Implements the grasp matrix. Note this is the inverse
 * of the transpose of the kinematics matrix, so maybe
 * implement it that way?
 */
public class SwerveDynamics {
    private static final boolean DEBUG = false;
    // This is unconstrained actuation so there are no
    // normal vectors here.
    /** Robot dynamics, to obtain the whole-robot wrench. */
    private final SE2Dynamics m_dyn;
    /** Tire slip angle model. */
    private final Tire m_tire;
    /** Inverse dynamics matrix. */
    private final Matrix<N8, N3> m_inv;

    /**
     * @param m  mass kg
     * @param I  inertia kg m^2
     * @param fl front-left contact
     * @param fr front-right contact
     * @param rl rear-left contact
     * @param rr rear-right contact
     */
    public SwerveDynamics(
            double m, double I,
            Tire tire,
            Translation2d fl, Translation2d fr,
            Translation2d rl, Translation2d rr) {
        m_dyn = new SE2Dynamics(m, I);
        m_tire = tire;
        Matrix<N3, N8> fwd = MatBuilder.fill(Nat.N3(), Nat.N8(),
                1, 0, 1, 0, 1, 0, 1, 0, //
                0, 1, 0, 1, 0, 1, 0, 1, //
                -fl.getY(), fl.getX(), -fr.getY(), fr.getX(), //
                -rl.getY(), rl.getX(), -rr.getY(), rr.getX());
        m_inv = new Matrix<>(fwd.getStorage().pseudoInverse());
    }

    /**
     * Returns corner efforts for the given rigid body acceleration.
     * 
     * @param states swerve module states from inverse kinematics (not measurement)
     * @param a      acceleration in robot frame.
     */
    public SwerveEffort effort(
            SwerveModuleStates states,
            ChassisAcceleration a) {
        CornerForces cornerForces = cornerForces(a);
        return new SwerveEffort(
                cornerEffort(states.frontLeft(), cornerForces.fl()),
                cornerEffort(states.frontRight(), cornerForces.fr()),
                cornerEffort(states.rearLeft(), cornerForces.rl()),
                cornerEffort(states.rearRight(), cornerForces.rr()));
    }

    /** Effort for a single corner. */
    ModuleEffort cornerEffort(
            SwerveModuleState100 state,
            CornerForce cornerForce) {
        // Total corner force, N
        Vector<N2> cornerForceVec = cornerForce.vector();
        Optional<Rotation2d> steeringAngle = state.angle();
        if (cornerForceVec.norm() < 1e-6) {
            if (DEBUG)
                System.out.println("no corner force => no effect");
            // There is no force to apply, so the effort is zero, with the
            // input angle.
            return new ModuleEffort(0, steeringAngle);
        }
        if (steeringAngle.isEmpty()) {
            if (DEBUG)
                System.out.println("no steering angle => all longitudinal");
            // If the angle does not exist, then the desired velocity is zero.
            // In that case, the actual steering angle should be determined
            // by the required force, which is all longitudinal.
            Rotation2d correctedAngle = GeometryUtil.rotation2dFromVec(cornerForceVec);
            return new ModuleEffort(
                    cornerForceVec.norm(),
                    Optional.of(correctedAngle));
        }

        // The angle exists, so the velocity is nonzero.
        // There is a defined longitudinal direction, so we decompose
        // the required force into longitudinal and lateral parts.
        Rotation2d steering = steeringAngle.get();
        // Which way the wheel is turning.
        double direction = Math.signum(state.speed());
        // Longitudinal normal (with correction for wheel direction).
        Vector<N2> longitudinalUnit = VecBuilder.fill(steering.getCos(), steering.getSin()).times(direction);
        // Longitudinal component, N
        Vector<N2> longitudinal = cornerForceVec.projection(longitudinalUnit);
        // Lateral component, N
        Vector<N2> lateral = cornerForceVec.minus(longitudinal);
        // Should the slip be to the left (+) or right (-)?
        // The determinant tells you.
        double slipDirection = Math.signum(GeometryUtil.det(longitudinalUnit, lateral));
        double slipAngle = m_tire.angle(lateral.norm()) * slipDirection;
        Rotation2d correctedAngle = steering.plus(new Rotation2d(slipAngle));
        return new ModuleEffort(longitudinal.norm(), Optional.of(correctedAngle));

    }

    /**
     * Corner forces for the given rigid body acceleration.
     * 
     * @param a acceleration in robot frame.
     */
    public CornerForces cornerForces(ChassisAcceleration a) {
        // Compute rigid body wrench.
        SE2Effort se2Effort = m_dyn.effort(a);
        Vector<N3> w = se2Effort.vector();
        // Find contact forces.
        Vector<N8> f = new Vector<>(m_inv.times(w));
        return CornerForces.fromVector(f);
    }

    /**
     * Planar (R2) force in Newtons.
     */
    record CornerForce(double x, double y) {
        Vector<N2> vector() {
            return VecBuilder.fill(x, y);
        }
    }

    record CornerForces(CornerForce fl, CornerForce fr, CornerForce rl, CornerForce rr) {
        /**
         * The argument is (f1x, f1y, f2x, f2y ...)
         * as specified in README.md.
         */
        public static CornerForces fromVector(Vector<N8> v) {
            return new CornerForces(
                    new CornerForce(v.get(0), v.get(1)),
                    new CornerForce(v.get(2), v.get(3)),
                    new CornerForce(v.get(4), v.get(5)),
                    new CornerForce(v.get(6), v.get(7)));
        }
    }
}
