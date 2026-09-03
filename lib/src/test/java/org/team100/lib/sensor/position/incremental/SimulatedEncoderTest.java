package org.team100.lib.sensor.position.incremental;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.sensor.position.incremental.sim.SimulatedEncoder;
import org.team100.lib.testing.Timeless;

public class SimulatedEncoderTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSetAndReset() {
        // changing the encoder position should not produce a velocity signal.

        SimulatedMotor motor = new SimulatedMotor(log, 600);
        SimulatedEncoder encoder = motor.encoder();
        assertEquals(0, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, motor.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, motor.getVelocityRad_S(), DELTA);

        encoder.setUnwrappedEncoderPositionRad(1);

        stepTime();

        assertEquals(1, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(1, motor.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, motor.getVelocityRad_S(), DELTA);

        stepTime();

        assertEquals(1, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(1, motor.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, motor.getVelocityRad_S(), DELTA);
    }

}
