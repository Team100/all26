package org.team100.lib.sensor.position.absolute.wpi;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class SimulatedAS5048Test implements Timeless {
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSquash() {
        Rotation2d r = Rotation2d.kZero;
        double dutyCycle = SimulatedAS5048.getDutyCycle(r);
        assertEquals(0.003888, dutyCycle, 0.000001);
        double turns = AS5048RotaryPositionSensor.mapSensorRange(
                dutyCycle,
                AS5048RotaryPositionSensor.SENSOR_MIN,
                AS5048RotaryPositionSensor.SENSOR_MAX);
        double angle = MathUtil.angleModulus(turns * Math.PI * 2);
        // Round-trip results in zero.
        assertEquals(0.000000, angle, 0.000001);
    }

    Rotation2d testR = null;

    @Test
    void testRoundTrip() {
        // Use a real sensor.
        AS5048RotaryPositionSensor sensor = new AS5048RotaryPositionSensor(
                log, new RoboRioChannel(1), 0, EncoderDrive.DIRECT);
        // This modifies the real sensor input.
        SimulatedAS5048 sim = new SimulatedAS5048((x) -> testR, sensor);

        // Input = 0.
        testR = new Rotation2d(0);
        sim.run();
        assertEquals(0.003888, sim.output(), 0.000001);
        // Sensor is only updated by the cache.
        stepTime();
        // Output = 0.
        assertEquals(0.000000, sensor.getWrappedPositionRad(), 0.000001);

        // Input = pi/2.
        testR = new Rotation2d(Math.PI / 2);
        sim.run();
        assertEquals(0.252430, sim.output(), 0.000001);
        // Sensor is only updated by the cache.
        stepTime();
        // Output = pi/2.
        assertEquals(Math.PI / 2, sensor.getWrappedPositionRad(), 0.001);
    }
}
