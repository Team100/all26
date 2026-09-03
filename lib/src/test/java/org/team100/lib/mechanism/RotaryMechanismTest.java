package org.team100.lib.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Friction;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motor.MockMotor;
import org.team100.lib.sensor.position.absolute.MockRotaryPositionSensor;
import org.team100.lib.testing.Timeless;

public class RotaryMechanismTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    /** Show that the limits have effect. */
    @Test
    void testLimits() {
        Friction friction = new Friction(0.100, 0.100, 0.0, 0.1);
        MockMotor motor = new MockMotor(friction);
        MockRotaryPositionSensor sensor = new MockRotaryPositionSensor();
        double gearRatio = 1;
        RotaryMechanism mech = new RotaryMechanism(logger, motor, sensor, gearRatio, 1, 2);

        // duty cycle limit observes the encoder
        // within bounds => ok
        sensor.angle = 1.5;
        mech.setDutyCycle(1.0);
        assertEquals(1.0, motor.output, DELTA);
        // out of bounds => stop.
        sensor.angle = 2.5;
        mech.setDutyCycle(1.0);
        assertEquals(0.0, motor.output, DELTA);

        // velocity limit observes the encoder
        // within bounds => ok
        sensor.angle = 1.5;
        mech.setVelocity(1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);
        // out of bounds => stop
        sensor.angle = 2.5;
        mech.setVelocity(1.0, 0);
        assertEquals(0.0, motor.velocity, DELTA);

        // positional limits filter the input
        // within bounds => ok
        mech.setUnwrappedPosition(1.5, 1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);
        // out of bounds => stop
        mech.setUnwrappedPosition(2.5, 1.0, 0);
        assertEquals(0.0, motor.velocity, DELTA);
    }

    /** Same cases as above, but unlimited */
    @Test
    void testUnlimited() {
        Friction friction = new Friction(0.100, 0.100, 0.0, 0.1);
        MockMotor motor = new MockMotor(friction);
        MockRotaryPositionSensor sensor = new MockRotaryPositionSensor();
        double gearRatio = 1;
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, gearRatio, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        // duty cycle limit observes the encoder
        // within bounds => ok
        sensor.angle = 1.5;
        mech.setDutyCycle(1.0);
        assertEquals(1.0, motor.output, DELTA);
        // out of bounds => stop.
        sensor.angle = 2.5;
        mech.setDutyCycle(1.0);
        assertEquals(1.0, motor.output, DELTA);

        // velocity limit observes the encoder
        // within bounds => ok
        sensor.angle = 1.5;
        mech.setVelocity(1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);
        // out of bounds => stop
        sensor.angle = 2.5;
        mech.setVelocity(1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);

        // positional limits filter the input
        // within bounds => ok
        mech.setUnwrappedPosition(1.5, 1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);
        // out of bounds => stop
        mech.setUnwrappedPosition(2.5, 1.0, 0);
        assertEquals(1.0, motor.velocity, DELTA);
    }

    @Test
    void testWrapNearMeasurement() {
        LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
        Friction friction = new Friction(0.100, 0.100, 0.0, 0.1);
        MockMotor motor = new MockMotor(friction);
        MockRotaryPositionSensor sensor = new MockRotaryPositionSensor();
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        // 0 -> 3
        assertEquals(0, mech.getWrappedPositionRad(), DELTA);
        assertEquals(0, mech.getUnwrappedPositionRad(), DELTA);
        // -3 -> 3 the short way around
        sensor.angle = -3;
        assertEquals(-3, mech.getWrappedPositionRad(), DELTA);
        assertEquals(-3, mech.getUnwrappedPositionRad(), DELTA);
        // -3 -> 1 the short way around
        sensor.angle = -3;
        assertEquals(-3, mech.getWrappedPositionRad(), DELTA);
        assertEquals(-3, mech.getUnwrappedPositionRad(), DELTA);
        // -pi/2 -> pi/2
        sensor.angle = -Math.PI / 2;
        assertEquals(-Math.PI / 2, mech.getWrappedPositionRad(), DELTA);
        assertEquals(-Math.PI / 2, mech.getUnwrappedPositionRad(), DELTA);
        // unwrapped case, -5pi/2 -> -3pi/2
        sensor.angle = -5 * Math.PI / 2;
        assertEquals(-Math.PI / 2, mech.getWrappedPositionRad(), DELTA);
        assertEquals(-5 * Math.PI / 2, mech.getUnwrappedPositionRad(), DELTA);
    }

}
