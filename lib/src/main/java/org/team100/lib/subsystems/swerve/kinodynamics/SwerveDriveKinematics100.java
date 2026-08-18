package org.team100.lib.subsystems.swerve.kinodynamics;

import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDelta;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleState100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

/**
 * The Jacobians of the swerve kinematics, relating velocities of the
 * wheels (qdot) to velocity of the whole robot (xdot).
 * 
 * We also use the Jacobian to relate small *differences* in wheels (dq), a
 * "delta", to *differences* in robot pose (dx), a "twist." For these cases,
 * we assume the wheel steering is constant across the difference, always using
 * the "end state" as the steering value.
 * 
 * So the definitions are:
 * 
 * speeds:
 * 
 * xdot: [vx; vy; omega]
 * qdot: [vx0; vy0; vx1; vy1; vx2; vy2; vx3; vy3]
 * 
 * differences:
 * 
 * dx: [dx; dy; dtheta]
 * dq: [dx0; dy0; dx1; dy1; dx2; dy2; dx3; dy3]
 * 
 * Don't mix these up!
 * 
 * For the swerve drive, it is easier to start with the inverse Jacobian
 * (robot -> wheels) and invert it to get the forward Jacobian.
 * 
 * Note: the Jacobians are relative to the robot reference frame, not the
 * field reference frame.
 * 
 * Note: forward kinematics is never more accurate than the gyro and we
 * absolutely cannot operate without a functional gyro, so we should use the
 * gyro instead. see https://github.com/Team100/all24/issues/350
 */
public class SwerveDriveKinematics100 {

    /**
     * Because the kinematics covered here are in terms of velocities, the
     * "forward kinematics matrix" is *the Jacobian*:
     * 
     * xdot = J qdot
     * 
     * where qdot is the velocity of the actuators (i.e. wheels), and xdot is
     * the velocity of the end-effector (i.e. center of the robot).
     * 
     * Because there are 8 degrees of freedom in the wheels, and three degrees
     * of freedom in the robot, this matrix is 3x8.
     * 
     * It ends up something like:
     * 
     * <pre>
     *  0.25 0.00 0.25 0.00 ...
     *  0.00 0.25 0.00 0.25 ...
     * -1.00 1.00 1.00 1.00 ...
     * </pre>
     * 
     * the last row depends on the drive dimensions.
     * 
     * so when multiplied by qdot, the result is xdot:
     * 
     * <pre>
     * mean(vx)
     * mean(vy)
     * some combination depending on dimensions
     * </pre>
     */
    final Matrix<N3, N8> m_J;

    /**
     * Because the kinematics covered here are in terms of velocities, the
     * "reverse kinematics matrix" is *the inverse Jacobian*:
     * 
     * qdot = Jinv xdot
     * 
     * where xdot is the velocity of the end-effector (i.e. center of the robot),
     * and qdot is the velocity of the actuators (i.e. wheels).
     * 
     * Because there are 8 degrees of freedom in the wheels, and three degrees
     * of freedom in the robot, this matrix is 3x8.
     * 
     * It ends up something like:
     * 
     * <pre>
     * 1   0  -y0
     * 0   1   x0
     * 1   0  -y1
     * 0   1   x1
     * ...
     * </pre>
     * 
     * when multiplied by xdot, the result is qdot:
     * 
     * <pre>
     * vx - y0 * omega
     * vy + x0 * omega
     * vx - y1 * omega
     * vy + x1 * omega
     * ...
     * </pre>
     * 
     * This vector is tranformed into the module states (or deltas)
     * 
     * Note: the states may include empty angles for motionless wheels
     * Note: the state speeds are always positive.
     */
    final Matrix<N8, N3> m_Jinv;

    /**
     * Translations are relative to the center (i.e. usually half the wheelbase).
     */
    public SwerveDriveKinematics100(
            Translation2d frontLeft,
            Translation2d frontRight,
            Translation2d rearLeft,
            Translation2d rearRight) {
        m_Jinv = Jinv(frontLeft, frontRight, rearLeft, rearRight);
        m_J = new Matrix<>(m_Jinv.getStorage().pseudoInverse());
    }

    public DiscreteSpeed forward(SwerveModuleStates states, double dt) {
        return speed(new Vector<>(m_J.times(qdot(states))), dt);
    }

    public Twist2d forward(SwerveModuleDeltas deltas) {
        return twist(new Vector<>(m_J.times(dq(deltas))));
    }

    public SwerveModuleStates inverse(DiscreteSpeed speed) {
        return states(new Vector<>(m_Jinv.times(xdot(speed))));
    }

    public SwerveModuleDeltas inverse(Twist2d twist) {
        return deltas(new Vector<>(m_Jinv.times(dx(twist))));
    }

    ///////////////////////////////////////

    private static Vector<N8> qdot(SwerveModuleStates states) {
        return VecBuilder.fill(
                states.frontLeft().vx(),
                states.frontLeft().vy(),
                states.frontRight().vx(),
                states.frontRight().vy(),
                states.rearLeft().vx(),
                states.rearLeft().vy(),
                states.rearRight().vx(),
                states.rearRight().vy());
    }

    private Vector<N8> dq(SwerveModuleDeltas deltas) {
        return VecBuilder.fill(
                deltas.frontLeft().dx(),
                deltas.frontLeft().dy(),
                deltas.frontRight().dx(),
                deltas.frontRight().dy(),
                deltas.rearLeft().dx(),
                deltas.rearLeft().dy(),
                deltas.rearRight().dx(),
                deltas.rearRight().dy());
    }

    private static Vector<N3> xdot(DiscreteSpeed speed) {
        return VecBuilder.fill(
                speed.twist().dx / speed.dt(),
                speed.twist().dy / speed.dt(),
                speed.twist().dtheta / speed.dt());
    }

    private static Vector<N3> dx(Twist2d twist) {
        return VecBuilder.fill(twist.dx, twist.dy, twist.dtheta);
    }

    private SwerveModuleStates states(Vector<N8> qdot) {
        return new SwerveModuleStates(
                SwerveModuleState100.fromSpeed(qdot.get(0), qdot.get(1)),
                SwerveModuleState100.fromSpeed(qdot.get(2), qdot.get(3)),
                SwerveModuleState100.fromSpeed(qdot.get(4), qdot.get(5)),
                SwerveModuleState100.fromSpeed(qdot.get(6), qdot.get(7)));
    }

    private SwerveModuleDeltas deltas(Vector<N8> dq) {
        return new SwerveModuleDeltas(
                new SwerveModuleDelta(dq.get(0), dq.get(1)),
                new SwerveModuleDelta(dq.get(2), dq.get(3)),
                new SwerveModuleDelta(dq.get(4), dq.get(5)),
                new SwerveModuleDelta(dq.get(6), dq.get(7)));
    }

    private DiscreteSpeed speed(Vector<N3> xdot, double dt) {
        return new DiscreteSpeed(new Twist2d(
                xdot.get(0) * dt,
                xdot.get(1) * dt,
                xdot.get(2) * dt), dt);
    }

    private Twist2d twist(Vector<N3> dx) {
        return new Twist2d(dx.get(0), dx.get(1), dx.get(2));
    }

    /**
     * Inverse Jacobian, 8x3, given module translations relative to the tool center
     * point (robot center).
     */
    private static Matrix<N8, N3> Jinv(
            Translation2d frontLeft,
            Translation2d frontRight,
            Translation2d rearLeft,
            Translation2d rearRight) {
        return MatBuilder.fill(Nat.N8(), Nat.N3(),
                1, 0, -1.0 * frontLeft.getY(),
                0, 1, frontLeft.getX(),
                1, 0, -1.0 * frontRight.getY(),
                0, 1, frontRight.getX(),
                1, 0, -1.0 * rearLeft.getY(),
                0, 1, rearLeft.getX(),
                1, 0, -1.0 * rearRight.getY(),
                0, 1, rearRight.getX());
    }
}
