package org.team100.lib.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Friction;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.MockMotor;
import org.team100.lib.profile.r1.AccelLimitedVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.NoVelocityReferenceR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.sensor.position.incremental.MockIncrementalEncoder;

class LinearVelocityServoTest {
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testNoReset() {
        PDynamics dyn = new PDynamics(0);
        Friction friction = new Friction(0.100, 0.100, 0.0, 0.1);
        MockMotor driveMotor = new MockMotor(friction);
        MockIncrementalEncoder driveEncoder = new MockIncrementalEncoder();
        LinearMechanism mech = new LinearMechanism(logger,
                driveMotor, driveEncoder, 1, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(10);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                logger, () -> profile, 1);
        OutboardLinearVelocityServo servo = new OutboardLinearVelocityServo(
                logger, mech, dyn, ref, 1);
        // 0.5 m/s
        servo.setVelocityProfiled(0.5);
    }

    @Test
    void testSimple() {
        PDynamics dyn = new PDynamics(0);
        Friction friction = new Friction(0.100, 0.100, 0.0, 0.1);
        MockMotor driveMotor = new MockMotor(friction);
        MockIncrementalEncoder driveEncoder = new MockIncrementalEncoder();
        LinearMechanism mech = new LinearMechanism(logger,
                driveMotor, driveEncoder, 1, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        OutboardLinearVelocityServo servo = new OutboardLinearVelocityServo(
                logger, mech, dyn, new NoVelocityReferenceR1(), 1);
        // 0.5 m/s
        servo.setVelocityDirect(0.5, 0);
        // wheel radius is 0.5 m, so drive speed is 1 m/s
        assertEquals(1.0, driveMotor.velocity, 0.001);
    }
}
