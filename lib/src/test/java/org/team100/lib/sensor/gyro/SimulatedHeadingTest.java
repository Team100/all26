package org.team100.lib.sensor.gyro;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.dynamics.swerve.SwerveEffort;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SimulatedHeadingTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TotalCurrentLog currentLog = new TotalCurrentLog(logger);

    @Test
    void testInitial() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forRealisticTest();
        SwerveModuleCollection c = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20), l);
        SimulatedGyro h = new SimulatedGyro(logger, l, c, 0);
        assertEquals(0, h.getYawNWU().getRadians(), DELTA);
        assertEquals(0, h.getYawRateNWU(), DELTA);
    }

    @Test
    void testTranslation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forRealisticTest();
        SwerveModuleCollection c = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20), l);
        SwerveModulePositions p = c.positions();
        assertEquals(0, p.frontLeft().distanceMeters(), DELTA);
        assertEquals(0, p.frontRight().distanceMeters(), DELTA);
        assertEquals(0, p.rearLeft().distanceMeters(), DELTA);
        assertEquals(0, p.rearRight().distanceMeters(), DELTA);
        SimulatedGyro h = new SimulatedGyro(logger, l, c, 0);
        ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 0);
        // includes discretization
        SwerveModuleStates states = l.toSwerveModuleStates(speeds);
        c.reset();
        stepTime();
        // go for 0.4s
        for (int i = 0; i < 20; ++i) {
            c.setDesiredStates(states, SwerveEffort.ZERO);
            stepTime();
        }
        assertEquals(0, h.getYawNWU().getRadians(), DELTA);
        assertEquals(0, h.getYawRateNWU(), DELTA);
        p = c.positions();
        assertEquals(0.42, p.frontLeft().distanceMeters(), 0.03);
        assertEquals(0.42, p.frontRight().distanceMeters(), 0.03);
        assertEquals(0.42, p.rearLeft().distanceMeters(), 0.03);
        assertEquals(0.42, p.rearRight().distanceMeters(), 0.03);
    }

    @Test
    void testRotation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forRealisticTest();
        SwerveModuleCollection c = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20), l);
        SimulatedGyro h = new SimulatedGyro(logger, l, c, 0);
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1);
        // includes discretization
        SwerveModuleStates states = l.toSwerveModuleStates(speeds);

        c.reset();
        // steering velocity is 13 rad/s, we need to go about 2 rad? so wait 0.2 sec?
        for (int i = 0; i < 20; ++i) {
            // get the modules pointing the right way (wait for the steering profiles)
            c.setDesiredStates(states, SwerveEffort.ZERO);
            if (DEBUG)
                System.out.printf("rotation %6.3f yaw %6.3f\n",
                        c.positions().frontLeft().unwrappedAngle().get().getRadians(),
                        h.getYawNWU().getRadians());
            stepTime();
        }

        // With setPositionDirect in SwerveModule100, this is 0.366, i.e. it responds
        // faster.
        // with setPositionProfiled, it's 0.286, i.e. slower.
        // assertEquals(0.336, h.getYawNWU().getRadians(), 0.03);
        // the rate is what we asked for.
        assertEquals(1, h.getYawRateNWU(), DELTA);
    }

    @Test
    void testHolonomic() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forRealisticTest();

        ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 1);
        // includes discretization
        SwerveModuleStates states = l.toSwerveModuleStates(speeds);
        // these are discretized so not symmetrical
        assertEquals(0.787, states.frontLeft().speed(), DELTA);
        assertEquals(1.273, states.frontRight().speed(), DELTA);
        assertEquals(0.794, states.rearLeft().speed(), DELTA);
        assertEquals(1.277, states.rearRight().speed(), DELTA);
        assertEquals(0.310, states.frontLeft().angle().get().getRadians(), DELTA);
        assertEquals(0.190, states.frontRight().angle().get().getRadians(), DELTA);
        assertEquals(-0.334, states.rearLeft().angle().get().getRadians(), DELTA);
        assertEquals(-0.205, states.rearRight().angle().get().getRadians(), DELTA);

        SwerveModuleCollection c = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20), l);
        SimulatedGyro h = new SimulatedGyro(logger, l, c, 0);
        c.reset();

        // steering velocity is 13 rad/s, we need to go about 2 rad? so wait 0.2 sec?
        for (int i = 0; i < 20; ++i) {
            // get the modules pointing the right way (wait for the steering profiles)
            c.setDesiredStates(states, SwerveEffort.ZERO);
            stepTime();
            h.getYawNWU();
        }
        SwerveModuleStates states2 = c.states();

        // we get back what we put in
        assertEquals(0.787, states2.frontLeft().speed(), DELTA);
        assertEquals(1.273, states2.frontRight().speed(), DELTA);
        assertEquals(0.794, states2.rearLeft().speed(), DELTA);
        assertEquals(1.277, states2.rearRight().speed(), DELTA);
        assertEquals(0.310, states2.frontLeft().angle().get().getRadians(), DELTA);
        assertEquals(0.190, states2.frontRight().angle().get().getRadians(), 0.01);
        assertEquals(-0.334, states2.rearLeft().angle().get().getRadians(), DELTA);
        assertEquals(-0.205, states2.rearRight().angle().get().getRadians(), 0.01);

        // we wanted to turn 1 rad/s for 0.4s so this is close.
        assertEquals(0.38, h.getYawNWU().getRadians(), 0.03);
        // we wanted to move 1 rad/s, so that's what we got.
        assertEquals(1.000, h.getYawRateNWU(), DELTA);
    }
}
