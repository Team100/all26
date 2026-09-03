package org.team100.lib.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Friction;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motor.MockMotor;
import org.team100.lib.sensor.position.incremental.MockIncrementalEncoder;
import org.team100.lib.testing.Timeless;

public class LinearMechanismTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    /** Show that the limits have effect. */
    @Test
    void testLimits() {
        Friction friction = new Friction(0.100, 0.100, 0.0, 0.1);
        MockMotor motor = new MockMotor(friction);
        MockIncrementalEncoder encoder = new MockIncrementalEncoder();
        double gearRatio = 1;
        double wheelDiameterM = 2;
        LinearMechanism mech = new LinearMechanism(
                logger, motor, encoder, gearRatio, wheelDiameterM, 1, 2);

        // duty cycle limit observes the encoder
        // within bounds => ok
        encoder.position = 1.5;
        mech.setDutyCycle(1.0);
        assertEquals(1.0, motor.output, DELTA);
        // out of bounds => stop.
        encoder.position = 2.5;
        mech.setDutyCycle(1.0);
        assertEquals(0.0, motor.output, DELTA);

        // velocity limit observes the encoder
        // within bounds => ok
        encoder.position = 1.5;
        mech.setVelocity(1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);
        // out of bounds => stop
        encoder.position = 2.5;
        mech.setVelocity(1.0, 0);
        assertEquals(0.0, motor.velocity, DELTA);

        // positional limits filter the input
        // within bounds => ok
        mech.setPosition(1.5, 1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);
        // out of bounds => stop
        mech.setPosition(2.5, 1.0, 0);
        assertEquals(0.0, motor.velocity, DELTA);
    }

    /** Same cases as above, but unlimited */
    @Test
    void testUnlimited() {
        Friction friction = new Friction(0.100, 0.100, 0.0, 0.1);
        MockMotor motor = new MockMotor(friction);
        MockIncrementalEncoder encoder = new MockIncrementalEncoder();
        double gearRatio = 1;
        double wheelDiameterM = 2;
        LinearMechanism mech = new LinearMechanism(logger,
                motor, encoder, gearRatio, wheelDiameterM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        // duty cycle limit observes the encoder
        // within bounds => ok
        encoder.position = 1.5;
        mech.setDutyCycle(1.0);
        assertEquals(1.0, motor.output, DELTA);
        // out of bounds => stop.
        encoder.position = 2.5;
        mech.setDutyCycle(1.0);
        assertEquals(1.0, motor.output, DELTA);

        // velocity limit observes the encoder
        // within bounds => ok
        encoder.position = 1.5;
        mech.setVelocity(1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);
        // out of bounds => stop
        encoder.position = 2.5;
        mech.setVelocity(1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);

        // positional limits filter the input
        // within bounds => ok
        mech.setPosition(1.5, 1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);
        // out of bounds => stop
        mech.setPosition(2.5, 1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);
    }

}
