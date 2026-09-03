package org.team100.lib.sensor.position.absolute;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Friction;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MockMotor;
import org.team100.lib.sensor.position.incremental.MockIncrementalEncoder;
import org.team100.lib.testing.Timeless;

class CombinedRotaryPositionSensorTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testZeroing() {
        Friction friction = new Friction(0.100, 0.100, 0.0, 0.1);
        MockMotor motor = new MockMotor(friction);

        // this is the "correct" value
        MockRotaryPositionSensor sensor = new MockRotaryPositionSensor();
        sensor.angle = 1;
        assertEquals(1, sensor.getWrappedPositionRad(), DELTA);

        // this value is the "incorrect" value, should be overwritten by the combined
        // encoder constructor.
        MockIncrementalEncoder encoder = new MockIncrementalEncoder();
        encoder.position = 0;
        assertEquals(0, encoder.getUnwrappedPositionRad(), DELTA);

        // the proxy just produces the modulus of the encoder.
        ProxyRotaryPositionSensor proxy = new ProxyRotaryPositionSensor(encoder, 1);
        assertEquals(0, proxy.getWrappedPositionRad(), DELTA);

        CombinedRotaryPositionSensor combined = new CombinedRotaryPositionSensor(logger, sensor, proxy);
        // the combined encoder has not been synchronized
        assertEquals(0.0, combined.getWrappedPositionRad(), DELTA);

        RotaryMechanism m = new RotaryMechanism(
                logger, motor, combined, 1.0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        // the mechanism shows the wrong position
        assertEquals(0.0, m.getWrappedPositionRad(), DELTA);

        combined.sync();
        // the combined encoder reads the correct value
        assertEquals(1.0, combined.getWrappedPositionRad(), DELTA);

        // and the secondary encoder has been "fixed"
        assertEquals(1.0, encoder.position, DELTA);

        // the mechanism shows the right position
        assertEquals(1.0, m.getWrappedPositionRad(), DELTA);
    }
}
