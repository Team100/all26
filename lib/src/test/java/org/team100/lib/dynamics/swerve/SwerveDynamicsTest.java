package org.team100.lib.dynamics.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.dynamics.swerve.SwerveDynamics.CornerForce;
import org.team100.lib.dynamics.swerve.SwerveDynamics.CornerForces;
import org.team100.lib.dynamics.swerve.SwerveEffort.ModuleEffort;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.subsystems.swerve.kinodynamics.DiscreteSpeed;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleState100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

public class SwerveDynamicsTest {

    Translation2d fl = new Translation2d(0.25, 0.25);
    Translation2d fr = new Translation2d(0.25, -0.25);
    Translation2d rl = new Translation2d(-0.25, 0.25);
    Translation2d rr = new Translation2d(-0.25, -0.25);

    // Confirm the pseudoinverse of the example.
    @Test
    void test0() {
        Matrix<N3, N8> m = MatBuilder.fill(Nat.N3(), Nat.N8(),
                1, 0, 1, 0, 1, 0, 1, 0, //
                0, 1, 0, 1, 0, 1, 0, 1, //
                -1, 1, 1, 1, -1, -1, 1, -1);
        Matrix<N8, N3> minv = new Matrix<>(m.getStorage().pseudoInverse());
        System.out.println(minv);
    }

    /** Corner forces for simple accel: all same, 1/4 each. */
    @Test
    void test1() {
        SwerveDynamics d = new SwerveDynamics(
                70, 6, new Tire(200, 0), fl, fr, rl, rr);
        ChassisAcceleration a = new ChassisAcceleration(1, 0, 0);
        CornerForces e = d.cornerForces(a);
        assertEquals(17.5, e.fl().x(), 0.001);
        assertEquals(0, e.fl().y(), 0.001);

        assertEquals(17.5, e.fr().x(), 0.001);
        assertEquals(0, e.fr().y(), 0.001);

        assertEquals(17.5, e.rl().x(), 0.001);
        assertEquals(0, e.rl().y(), 0.001);

        assertEquals(17.5, e.rr().x(), 0.001);
        assertEquals(0, e.rr().y(), 0.001);
    }

    @Test
    void test1a() {
        SwerveDynamics d = new SwerveDynamics(
                70, 6, new Tire(200, 0), fl, fr, rl, rr);
        ChassisAcceleration a = new ChassisAcceleration(0, 1, 0);
        CornerForces e = d.cornerForces(a);
        assertEquals(0, e.fl().x(), 0.001);
        assertEquals(17.5, e.fl().y(), 0.001);

        assertEquals(0, e.fr().x(), 0.001);
        assertEquals(17.5, e.fr().y(), 0.001);

        assertEquals(0, e.rl().x(), 0.001);
        assertEquals(17.5, e.rl().y(), 0.001);

        assertEquals(0, e.rr().x(), 0.001);
        assertEquals(17.5, e.rr().y(), 0.001);
    }

    /** Efforts for zero accel => zero effort */
    @Test
    void test2() {
        SwerveDynamics d = new SwerveDynamics(
                70, 6, new Tire(200, 0), fl, fr, rl, rr);
        SwerveDriveKinematics100 k = new SwerveDriveKinematics100(
                fl, fr, rl, rr);
        // 1 m/s +x
        DiscreteSpeed v = new DiscreteSpeed(new Twist2d(0.02, 0, 0), 0.02);
        SwerveModuleStates s = k.inverse(v);
        assertEquals(1, s.frontLeft().speed(), 0.001);
        assertEquals(0, s.frontLeft().angle().get().getRadians(), 0.001);
        // no accel
        AccelerationSE2 a = new AccelerationSE2(0, 0, 0);
        ChassisAcceleration accel = ChassisAcceleration.fromFieldRelative(
                a, Rotation2d.kZero);
        SwerveEffort e = d.effort(s, accel);
        assertEquals(0, e.fl().f(), 0.001);
        assertEquals(0, e.fl().angle().get().getRadians(), 0.001);
    }

    /** Effort for longitudinal accel */
    @Test
    void test3() {
        SwerveDynamics d = new SwerveDynamics(
                70, 6, new Tire(200, 0), fl, fr, rl, rr);
        SwerveDriveKinematics100 k = new SwerveDriveKinematics100(
                fl, fr, rl, rr);
        // 1 m/s +x
        DiscreteSpeed v = new DiscreteSpeed(new Twist2d(0.02, 0, 0), 0.02);
        SwerveModuleStates s = k.inverse(v);
        assertEquals(1, s.frontLeft().speed(), 0.001);
        assertEquals(0, s.frontLeft().angle().get().getRadians(), 0.001);
        // longitudinal accel
        AccelerationSE2 a = new AccelerationSE2(1, 0, 0);
        ChassisAcceleration accel = ChassisAcceleration.fromFieldRelative(
                a, Rotation2d.kZero);
        SwerveEffort e = d.effort(s, accel);
        // one-quarter of the required force in the longitudinal direction
        assertEquals(17.5, e.fl().f(), 0.001);
        assertEquals(0, e.fl().angle().get().getRadians(), 0.001);
    }

    /** rotated, so field-relative +x resolves as robot +y */
    @Test
    void test3a() {
        SwerveDynamics d = new SwerveDynamics(
                70, 6, new Tire(200, 0.05), fl, fr, rl, rr);
        SwerveDriveKinematics100 k = new SwerveDriveKinematics100(
                fl, fr, rl, rr);
        // 1 m/s +x (really -y)
        DiscreteSpeed v = new DiscreteSpeed(new Twist2d(0.02, 0, 0), 0.02);
        SwerveModuleStates s = k.inverse(v);
        assertEquals(1, s.frontLeft().speed(), 0.001);
        assertEquals(0, s.frontLeft().angle().get().getRadians(), 0.001);
        // x accel (+y robot-relative)
        AccelerationSE2 a = new AccelerationSE2(1, 0, 0);
        // -pi/2 rotation
        ChassisAcceleration accel = ChassisAcceleration.fromFieldRelative(
                a, Rotation2d.kCW_Pi_2);
        SwerveEffort e = d.effort(s, accel);
        // no longitudinal
        assertEquals(0, e.fl().f(), 0.001);
        // slip
        assertEquals(0.004, e.fl().angle().get().getRadians(), 0.001);
    }

    /**
     * Effort for lateral accel, with a non-slipping tire => doesn't do anything.
     */
    @Test
    void test4() {
        SwerveDynamics d = new SwerveDynamics(
                70, 6, new Tire(200, 0), fl, fr, rl, rr);
        SwerveDriveKinematics100 k = new SwerveDriveKinematics100(
                fl, fr, rl, rr);
        // 1 m/s +x
        DiscreteSpeed v = new DiscreteSpeed(new Twist2d(0.02, 0, 0), 0.02);
        SwerveModuleStates s = k.inverse(v);
        assertEquals(1, s.frontLeft().speed(), 0.001);
        assertEquals(0, s.frontLeft().angle().get().getRadians(), 0.001);
        // lateral accel +y
        AccelerationSE2 a = new AccelerationSE2(0, 1, 0);
        ChassisAcceleration accel = ChassisAcceleration.fromFieldRelative(
                a, Rotation2d.kZero);
        SwerveEffort e = d.effort(s, accel);
        // the wheel headed in +x is incapable of providing
        // longitudinal force in the correct direction,
        // so the extra motor torque will be zero.
        assertEquals(0, e.fl().f(), 0.001);
        // there's no wheel slip allowed, so there's no lateral force :(
        assertEquals(0, e.fl().angle().get().getRadians(), 0.000001);
    }

    /** A tire that does not slip cannot produce corner force */
    @Test
    void test5() {
        SwerveDynamics d = new SwerveDynamics(
                70, 6, new Tire(200, 0), fl, fr, rl, rr);

        SwerveModuleState100 s = new SwerveModuleState100(1, Optional.of(Rotation2d.kZero));
        CornerForce cf = new CornerForce(0, 17.5); // lateral accel +y
        ModuleEffort e = d.cornerEffort(s, cf);

        // the wheel headed in +x is incapable of providing
        // longitudinal force in the correct direction,
        // so the extra motor torque will be zero.
        assertEquals(0, e.f(), 0.001);
        assertEquals(0, e.angle().get().getRadians(), 0.000001);
    }

    /** A tire with slip */
    @Test
    void test5a() {
        SwerveDynamics d = new SwerveDynamics(
                70, 6, new Tire(200, 0.05), fl, fr, rl, rr);

        SwerveModuleState100 s = new SwerveModuleState100(1, Optional.of(Rotation2d.kZero));
        CornerForce cf = new CornerForce(0, 17.5); // lateral force +y
        ModuleEffort e = d.cornerEffort(s, cf);

        // the wheel headed in +x is incapable of providing
        // longitudinal force in the correct direction,
        // so the extra motor torque will be zero.
        assertEquals(0, e.f(), 0.001);
        // toe-in 0.2 degrees
        assertEquals(0.004, e.angle().get().getRadians(), 0.001);
    }

}
