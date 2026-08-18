package org.team100.lib.sensor.position.absolute.wpi;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Cache;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;

/** Uses DutyCycleSim. */
public class DutyCycleRotaryPositionSensorTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void testSimple() {
        Cache.clear();
        LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
        AS5048RotaryPositionSensor sensor = new AS5048RotaryPositionSensor(
                log, new RoboRioChannel(0), 0, EncoderDrive.DIRECT);

        try {
            DutyCycleSim sim = new DutyCycleSim(sensor.getDutyCycle());
            sim.setFrequency(1000);
            sim.setInitialized(true);

            sim.setOutput(0.5);

            Cache.refresh();
            assertEquals(0.5, sensor.getRatio(), DELTA);
            assertEquals(3.135, sensor.getWrappedPositionRad(), DELTA);

            sim.setOutput(0.8);
            // nothing happens until the cache refreshes
            assertEquals(0.5, sensor.getRatio(), DELTA);
            assertEquals(3.135, sensor.getWrappedPositionRad(), DELTA);

            Cache.refresh();
            assertEquals(0.8, sensor.getRatio(), DELTA);
            assertEquals(-1.252, sensor.getWrappedPositionRad(), DELTA);
            // we have traversed in the positive direction
            assertEquals(1, sensor.getTurns());

            sim.setOutput(0.25);
            Cache.refresh();
            assertEquals(0.25, sensor.getRatio(), DELTA);
            assertEquals(1.555, sensor.getWrappedPositionRad(), DELTA);
            // this goes through zero, no wrap.
            assertEquals(1, sensor.getTurns());
        } finally {
            sensor.close();
        }
    }

    @Test
    void testWrapping() {
        Cache.clear();
        LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
        AS5048RotaryPositionSensor sensor = new AS5048RotaryPositionSensor(
                log, new RoboRioChannel(0), 0, EncoderDrive.DIRECT);
        try {
            DutyCycleSim sim = new DutyCycleSim(sensor.getDutyCycle());
            sim.setFrequency(1000);
            sim.setInitialized(true);

            final double range = sensor.sensorMax() - sensor.sensorMin();
            for (double unwrappedRad = -6 * Math.PI; unwrappedRad < 6 * Math.PI; unwrappedRad += 0.1) {
                double wrapped = MathUtil.inputModulus(unwrappedRad, 0, 2 * Math.PI);
                double piwrapped = MathUtil.angleModulus(unwrappedRad);
                double sensorTurns = wrapped / (2 * Math.PI);
                double ratio = sensorTurns * range + sensor.sensorMin();
                sim.setOutput(ratio);
                Cache.refresh();
                double sensorWrapped = sensor.getWrappedPositionRad();
                double turns = sensor.getTurns();
                // subtract the starting point so the wrapped numbers are the same.
                double sensorUnwrapped = sensor.getUnwrappedPositionRad() - 6 * Math.PI;
                if (DEBUG)
                    System.out.printf(
                            "unwrapped %6.3f wrapped %6.3f piwrapped %6.3f sensorTurns %6.3f ratio %6.3f sensorWrapped %6.3f turns %6.3f sensorUnwrapped %6.3f\n",
                            unwrappedRad, wrapped, piwrapped, sensorTurns, ratio, sensorWrapped, turns,
                            sensorUnwrapped);
                assertEquals(piwrapped, sensorWrapped, DELTA);
                assertEquals(unwrappedRad, sensorUnwrapped, DELTA);
                assertEquals(
                        sensor.getWrappedPositionRad(), MathUtil.angleModulus(sensor.getUnwrappedPositionRad()), DELTA);
            }
        } finally {
            sensor.close();
        }
    }
}
