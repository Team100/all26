package org.team100.lib.kinematics.prr;

import java.util.function.Function;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.prr.PRRAcceleration;
import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.geometry.prr.PRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.optimization.NumericalJacobian100;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

/**
 * Kinematics coordinates are as follows:
 * 
 * x axis is pointing up
 * y axis is pointing to the left
 * 
 * shoulder height is along x, relative to the origin
 * shoulder angle is relative to straight-up, positive counterclockwise
 * wrist angle is relative to the arm, positive counterclockwise
 * 
 * I chose these coordinates so that none of the angles ever transit the
 * pi/-pi boundary, which makes control simpler.
 * 
 * To avoid discontinuity, this now always uses the "reach up" direction, never
 * the "reach down" direction.
 */

public class PRRKinematics {
    public enum Solver {
        ANALYTIC, NUMERIC
    };

    /** length of link 2 (medial, arm) */
    private final double l2;
    /** length of link 3 (distal, finger) */
    private final double l3;
    /** Which method to use */
    private final Solver m_solver;
    /** Vector version of the forward kinematics */
    private final Function<Vector<N3>, Vector<N3>> m_f;

    /**
     * @param l2 length of link 2 (medial, arm)
     * @param l3 length of link 3 (distal, finger)
     */
    public PRRKinematics(double l2, double l3, Solver solver) {
        this.l2 = l2;
        this.l3 = l3;
        m_solver = solver;
        m_f = q -> GeometryUtil.toVec(forward(PRRConfig.fromVector(q)));
    }

    /**
     * Forward position kinematics: cartesian pose from joint config.
     * 
     * Pose is relative to the mech origin; remember to transform it to robot
     * coordinates.
     */
    public Pose2d forward(PRRConfig config) {
        double q1 = config.q1();
        double q2 = config.q2();
        double q3 = config.q3();
        double c2 = Math.cos(q2);
        double s2 = Math.sin(q2);
        double c23 = Math.cos(q2 + q3);
        double s23 = Math.sin(q2 + q3);
        double x = q1 + l2 * c2 + l3 * c23;
        double y = l2 * s2 + l3 * s23;
        double r = q2 + q3;
        return new Pose2d(x, y, new Rotation2d(r));
    }

    /**
     * Forward velocity kinematics: cartesian velocity from joint config and
     * velocity.
     * 
     * \dot{x} = J(q) \dot{q}
     * 
     * See doc/README.md equation 4
     */
    public VelocitySE2 forward(PRRConfig q, PRRVelocity qdot) {
        Matrix<N3, N3> J = J(q);
        return VelocitySE2.fromVector(J.times(qdot.toVector()));
    }

    /**
     * Forward acceleration kinematics: cartesian accel from joint config, velocity,
     * and accel
     * 
     * \ddot{x} = \dot{J} \dot{q} + J \ddot{q}
     * 
     * See doc/README.md equation 6
     */
    public AccelerationSE2 forward(
            PRRConfig q, PRRVelocity qdot, PRRAcceleration qddot) {
        Matrix<N3, N3> J = J(q);
        Matrix<N3, N3> Jdot = Jdot(q, qdot);
        return AccelerationSE2.fromVector(
                Jdot.times(qdot.toVector()).plus(J.times(qddot.toVector())));
    }

    /** Distance from shoulder pivot to wrist pivot. */
    public double armX(Translation2d wrist) {
        double d = l2 * l2 - wrist.getY() * wrist.getY();
        if (d < 0) {
            // Arm is horizontal.
            return 0;
        }
        return Math.sqrt(d);
    }

    /**
     * Inverse position kinematics: joint config from cartesian pose.
     * 
     * TODO: return a list here.
     */
    public PRRConfig inverse(Pose2d pose) {
        /** Translation from wrist axis to tool point. */
        Translation2d wristToTip = new Translation2d(l3, pose.getRotation());

        /** Location of wrist axis. */
        Translation2d wrist = pose.getTranslation().minus(wristToTip);

        /** Translation from shoulder axis to wrist axis. */
        Translation2d shoulderToWrist = new Translation2d(armX(wrist), wrist.getY());

        /** Location of shoulder axis. */
        Translation2d shoulder = wrist.minus(shoulderToWrist);

        /** Shoulder angle from vertical to arm. */
        Rotation2d shoulderAngle = shoulderToWrist.getAngle();

        /** Wrist angle from arm to tool point. */
        Rotation2d wristAngle = pose.getRotation().minus(shoulderToWrist.getAngle());

        return new PRRConfig(
                shoulder.getX(),
                shoulderAngle.getRadians(),
                wristAngle.getRadians());
    }

    /**
     * Inverse velocity kinematics: joint config from cartesian position.
     * 
     * \dot{q} = J^{-1} \dot{x}
     * 
     * See README.md equation 5
     */
    public PRRVelocity inverse(ModelSE2 m) {
        Pose2d x = m.pose();
        VelocitySE2 xdot = m.velocity();
        PRRConfig q = inverse(x);
        Matrix<N3, N3> Jinv = Jinv(q);
        return PRRVelocity.fromVector(Jinv.times(xdot.toVector()));
    }

    /**
     * Inverse acceleration kinematics: joint accel from cartesian position,
     * velocity, and accel.
     * 
     * \ddot{q} = J^{-1} (\ddot{x} - \dot{J} J^{-1} \dot{x})
     * 
     * See doc/README.md equation 9
     */
    public PRRAcceleration inverse(ControlSE2 m) {
        Pose2d x = m.pose();
        VelocitySE2 xdot = m.velocity();
        AccelerationSE2 xddot = m.acceleration();
        PRRConfig q = inverse(x);
        Matrix<N3, N3> Jinv = Jinv(q);
        PRRVelocity qdot = PRRVelocity.fromVector(Jinv.times(xdot.toVector()));
        Matrix<N3, N3> Jdot = Jdot(q, qdot);
        return PRRAcceleration.fromVector(
                Jinv.times(
                        xddot.toVector().minus(
                                Jdot.times(Jinv.times(xdot.toVector())))));
    }

    /**
     * End-effector Jacobian.
     */
    private Matrix<N3, N3> J(PRRConfig c) {
        return switch (m_solver) {
            case ANALYTIC -> analyticJ(c);
            case NUMERIC -> numericJ(c);
        };
    }

    /**
     * Inverse Jacobian.
     * 
     * When singular, some motion is still possible, so this doesn't return zero,
     * just the pseudoinverse. Note this might not be what you want?
     */
    private Matrix<N3, N3> Jinv(PRRConfig q) {
        Matrix<N3, N3> J = J(q);
        if (Math.abs(J.det()) < 1e-3) {
            System.out.printf("WARNING: singularity at config %s\n", q.toString());
        }
        return new Matrix<>(J.getStorage().pseudoInverse());
    }

    private Matrix<N3, N3> numericJ(PRRConfig c) {
        return NumericalJacobian100.numericalJacobian(
                Nat.N3(), Nat.N3(), m_f, c.toVector());
    }

    /**
     * End-effector Jacobian.
     * 
     * See doc/README.md equation 3
     */
    private Matrix<N3, N3> analyticJ(PRRConfig q) {
        double q2 = q.q2();
        double q3 = q.q3();
        double s2 = Math.sin(q2);
        double c2 = Math.cos(q2);
        double s23 = Math.sin(q2 + q3);
        double c23 = Math.cos(q2 + q3);
        return MatBuilder.fill(Nat.N3(), Nat.N3(), //
                1, -l2 * s2 - l3 * s23, -l3 * s23, //
                0, l2 * c2 + l3 * c23, l3 * c23, //
                0, 1, 1);
    }

    /**
     * Time-derivative of the end-effector Jacobian.
     * 
     * See doc/README.md equation 7
     */
    private Matrix<N3, N3> Jdot(PRRConfig q, PRRVelocity qdot) {
        double q2 = q.q2();
        double q3 = q.q3();
        double s2 = Math.sin(q2);
        double c2 = Math.cos(q2);
        double s23 = Math.sin(q2 + q3);
        double c23 = Math.cos(q2 + q3);
        double q2dot = qdot.q2dot();
        double q3dot = qdot.q3dot();
        return MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, -l2 * c2 * q2dot - l3 * c23 * (q2dot + q3dot), -l3 * c23 * (q2dot + q3dot), //
                0, -l2 * s2 * q2dot - l3 * s23 * (q2dot + q3dot), -l3 * s23 * (q2dot + q3dot), //
                0, 0, 0);
    }
}
