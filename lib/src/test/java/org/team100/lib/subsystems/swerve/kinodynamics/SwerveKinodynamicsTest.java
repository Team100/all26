package org.team100.lib.subsystems.swerve.kinodynamics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.dynamics.swerve.Tire;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveKinodynamicsTest implements Timeless {
    private static final double DELTA = 0.001;

    /** From field relative speed to robot relative speed to modules and back. */
    @Test
    void testRoundTripMotionless() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        VelocitySE2 v = new VelocitySE2(0, 0, 0);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds instantaneous = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);
        SwerveModuleStates states = unlimited.toSwerveModuleStates(instantaneous, 0.02);
        ChassisSpeeds implied = unlimited.toChassisSpeedsWithDiscretization(states, 0.02);
        VelocitySE2 result = SwerveKinodynamics.fromInstantaneousChassisSpeeds(implied, theta);
        assertEquals(0, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }

    /** From field relative speed to robot relative speed to modules and back. */
    @Test
    void testRoundTripDriveAndSpin() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        VelocitySE2 v = new VelocitySE2(5, 0, 25);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds instantaneous = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);
        SwerveModuleStates states = unlimited.toSwerveModuleStates(instantaneous, 0.02);
        ChassisSpeeds implied = unlimited.toChassisSpeedsWithDiscretization(states, 0.02);
        VelocitySE2 result = SwerveKinodynamics.fromInstantaneousChassisSpeeds(implied, theta);
        assertEquals(5, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(25, result.theta(), DELTA);
    }

    @Test
    void testComputedValues() {
        double track = 0.5;
        double wheelbase = 0.5;
        double driveV = 1;
        SwerveKinodynamics k = new SwerveKinodynamics(
                driveV, 1, 1, 1, track, track, wheelbase, wheelbase / 2, 1,
                70, 6, new Tire(175, 0.05));
        assertEquals(1, k.getMaxDriveVelocityM_S(), DELTA);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.353, r, DELTA);

        double omega = driveV / r;
        assertEquals(2.828, omega, DELTA);
        assertEquals(2.828, k.getMaxAngleSpeedRad_S(), DELTA);
    }

    @Test
    void testComputedValues2() {
        double track = 0.5;
        double wheelbase = 0.5;
        double driveV = 4;
        SwerveKinodynamics k = new SwerveKinodynamics(
                driveV, 1, 1, 1, track, track, wheelbase, wheelbase / 2, 1,
                70, 6, new Tire(175, 0.05));
        assertEquals(4, k.getMaxDriveVelocityM_S(), DELTA);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.353, r, DELTA);

        double omega = driveV / r;
        assertEquals(11.313, omega, DELTA);
        assertEquals(11.313, k.getMaxAngleSpeedRad_S(), DELTA);
    }

    @Test
    void testComputedValues3() {
        double track = 1;
        double wheelbase = 1;
        double driveV = 4;
        SwerveKinodynamics k = new SwerveKinodynamics(
                driveV, 1, 1, 1, track, track, wheelbase,
                wheelbase / 2, 1, 70, 6, new Tire(175, 0.05));
        assertEquals(4, k.getMaxDriveVelocityM_S(), DELTA);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.707, r, DELTA);

        double omega = driveV / r;
        assertEquals(5.656, omega, DELTA);
        assertEquals(5.656, k.getMaxAngleSpeedRad_S(), DELTA);
    }

    @Test
    void testComputedAngularAcceleration() {
        double track = 0.5;
        double wheelbase = 0.5;
        double driveA = 1;
        SwerveKinodynamics k = new SwerveKinodynamics(
                1, 1, driveA, 1, track, track, wheelbase, wheelbase / 2, 1,
                70, 6, new Tire(175, 0.05));
        assertEquals(1, k.getMaxDriveAccelerationM_S2(), DELTA);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.353, r, DELTA);

        double omegaDot = 12 * driveA * r / (track * track + wheelbase * wheelbase);

        assertEquals(8.485, omegaDot, DELTA);
        assertEquals(8.485, k.getMaxAngleAccelRad_S2(), DELTA);
    }

    @Test
    void testComputedAngularAcceleration2() {
        double track = 1;
        double wheelbase = 1;
        double driveA = 1;
        SwerveKinodynamics k = new SwerveKinodynamics(
                1, 1, driveA, 1, track, track, wheelbase, wheelbase / 2, 1,
                70, 6, new Tire(175, 0.05));
        assertEquals(1, k.getMaxDriveAccelerationM_S2(), DELTA);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.707, r, DELTA);

        double omegaDot = 12 * driveA * r / (track * track + wheelbase * wheelbase);

        // scales inverse with size
        assertEquals(4.242, omegaDot, DELTA);
        assertEquals(4.242, k.getMaxAngleAccelRad_S2(), DELTA);
    }

    @Test
    void testComputedCapsize() {
        double track = 1;
        double wheelbase = 1;
        double vcg = 0.3;
        SwerveKinodynamics k = new SwerveKinodynamics(
                1, 1, 1, 1, track, track, wheelbase, wheelbase / 2, vcg,
                70, 6, new Tire(175, 0.05));
        assertEquals(1, k.getMaxDriveAccelerationM_S2(), DELTA);

        double fulcrum = Math.min(track / 2, wheelbase / 2);
        assertEquals(0.5, fulcrum, DELTA);

        double accel = 9.8 * fulcrum / vcg;

        assertEquals(16.333, accel, DELTA);
        assertEquals(16.333, k.getMaxCapsizeAccelM_S2(), DELTA);
    }

    @Test
    void testAFewCases() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();
        double maxV = limits.getMaxDriveVelocityM_S();
        double maxOmega = limits.getMaxAngleSpeedRad_S();
        assertEquals(5, maxV, DELTA);
        assertEquals(14.142, maxOmega, DELTA);

        {
            // with no translation the wheel speed is ok
            ChassisSpeeds s = new ChassisSpeeds(0, 0, -9.38);
            // now discretizes.
            SwerveModuleStates ms = limits.toSwerveModuleStates(s);
            assertEquals(3.316, ms.frontLeft().speed(), DELTA);
            assertEquals(3.316, ms.frontRight().speed(), DELTA);
            assertEquals(3.316, ms.rearLeft().speed(), DELTA);
            assertEquals(3.316, ms.rearRight().speed(), DELTA);
        }
        {
            // with an extra ~2m/s, it's too fast
            ChassisSpeeds s = new ChassisSpeeds(0.13, -1.95, -9.38);
            SwerveModuleStates ms = limits.toSwerveModuleStates(s);
            // we no longer desaturate at this level: use the setpoint generator if you want
            // that.
            assertEquals(5.035, ms.frontLeft().speed(), DELTA);
            assertEquals(4.735, ms.frontRight().speed(), DELTA);
            assertEquals(2.689, ms.rearLeft().speed(), DELTA);
            assertEquals(2.074, ms.rearRight().speed(), DELTA);

            ChassisSpeeds i = limits.toChassisSpeedsWithDiscretization(ms, 0.02);
            // we get back what we put in
            assertEquals(0.13, i.vxMetersPerSecond, DELTA);
            assertEquals(-1.95, i.vyMetersPerSecond, DELTA);
            assertEquals(-9.38, i.omegaRadiansPerSecond, DELTA);
        }

    }

    @Test
    void testDiscretizationNoEffect() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forRealisticTest();
        // for this test the gyro rate and the commanded omega are the same,
        // though this is definitely not true in general
        {
            // pure rotation involves no discretization effect
            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeedsWithDiscretization(states, 0.02);
            assertEquals(0, impliedSpeeds.vxMetersPerSecond, DELTA);
            assertEquals(0, impliedSpeeds.vyMetersPerSecond, DELTA);
            assertEquals(1, impliedSpeeds.omegaRadiansPerSecond, DELTA);
        }
        {
            // pure translation involves no discretization effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 0);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeedsWithDiscretization(states, 0.02);
            assertEquals(1, impliedSpeeds.vxMetersPerSecond, DELTA);
            assertEquals(0, impliedSpeeds.vyMetersPerSecond, DELTA);
            assertEquals(0, impliedSpeeds.omegaRadiansPerSecond, DELTA);
        }
    }

    @Test
    void testDiscretizationWithEffect() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forRealisticTest();
        // for this test the gyro rate and the commanded omega are the same,
        // though this is definitely not true in general
        {
            // holonomic does have discretization effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 1);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeedsWithDiscretization(states, 0.02);
            assertEquals(1, impliedSpeeds.vxMetersPerSecond, DELTA);
            assertEquals(0, impliedSpeeds.vyMetersPerSecond, DELTA);
            assertEquals(1, impliedSpeeds.omegaRadiansPerSecond, DELTA);

            // invert the discretization to extract the original speeds
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(states, 0.02);
            assertEquals(0.999, correctedImplied.vxMetersPerSecond, DELTA);
            assertEquals(0, correctedImplied.vyMetersPerSecond, DELTA);
            assertEquals(1, correctedImplied.omegaRadiansPerSecond, DELTA);
        }
        {
            // more spinning => bigger effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 3);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeedsWithDiscretization(states, 0.02);
            assertEquals(1, impliedSpeeds.vxMetersPerSecond, DELTA);
            assertEquals(0, impliedSpeeds.vyMetersPerSecond, DELTA);
            assertEquals(3, impliedSpeeds.omegaRadiansPerSecond, DELTA);

            // invert the discretization to extract the original speeds.
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(states, 0.02);
            assertEquals(1, correctedImplied.vxMetersPerSecond, DELTA);
            assertEquals(0, correctedImplied.vyMetersPerSecond, DELTA);
            assertEquals(3, correctedImplied.omegaRadiansPerSecond, DELTA);
        }
        {
            // longer time interval => bigger effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 3);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.2);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeedsWithDiscretization(states, 0.2);
            assertEquals(1, impliedSpeeds.vxMetersPerSecond, DELTA);
            assertEquals(0, impliedSpeeds.vyMetersPerSecond, DELTA);
            assertEquals(3, impliedSpeeds.omegaRadiansPerSecond, DELTA);

            // invert the discretization to extract the original speeds.
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(states, 0.2);
            assertEquals(1, correctedImplied.vxMetersPerSecond, DELTA);
            assertEquals(0, correctedImplied.vyMetersPerSecond, DELTA);
            assertEquals(3, correctedImplied.omegaRadiansPerSecond, DELTA);
        }
        {
            // longer time interval => bigger effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 3);
            SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0.2);
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(states, 0.2);
            assertEquals(1, correctedImplied.vxMetersPerSecond, DELTA);
            assertEquals(0, correctedImplied.vyMetersPerSecond, DELTA);
            assertEquals(3, correctedImplied.omegaRadiansPerSecond, DELTA);
        }
    }

}
