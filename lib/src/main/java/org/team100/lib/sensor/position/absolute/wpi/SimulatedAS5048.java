package org.team100.lib.sensor.position.absolute.wpi;

import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Takt;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;

/**
 * Uses a function to supply ground-truth values for a time in the
 * recent past, and uses WPI simulation to make the real sensor input reflect
 * the ground-truth.
 */
public class SimulatedAS5048 implements Runnable {
    private static final boolean DEBUG = false;

    /**
     * The position sensor is assumed to have a fixed delay of 600 us.
     */
    private static final double DELAY_S = 0.0006;

    /**
     * Use the real sensor range.
     */
    private static final double SENSOR_RANGE = AS5048RotaryPositionSensor.SENSOR_MAX
            - AS5048RotaryPositionSensor.SENSOR_MIN;

    private final DoubleFunction<Rotation2d> m_truth;
    private final AS5048RotaryPositionSensor m_sensor;
    private final DutyCycleSim m_sim;

    public SimulatedAS5048(
            DoubleFunction<Rotation2d> truth,
            AS5048RotaryPositionSensor sensor) {
        m_truth = truth;
        m_sensor = sensor;
        // WPI "sim" classes poke values into the "real" classes.
        m_sim = new DutyCycleSim(sensor.getDutyCycle());
        m_sim.setInitialized(true);
        m_sim.setFrequency(1000);
        m_sim.setOutput(0.5);
    }

    @Override
    public void run() {
        // Sample the ground-truth value in the past.
        Rotation2d pastValue = m_truth.apply(Takt.actual() - DELAY_S);
        // Squash the value so that the real sensor can unsquash it.
        double dutyCycle = getDutyCycle(pastValue);
        // Set the HAL to the duty cycle for this rotation.
        m_sim.setOutput(dutyCycle);
        // This updates immediately (the numbers below match).
        if (DEBUG) {
            System.out.printf("SimulatedAS5048 pastValue %s\n", pastValue);
            double sensorDutyCycle = m_sensor.getDutyCycle().getOutput();
            System.out.printf("desired duty %f actual %f\n",
                    dutyCycle, sensorDutyCycle);
        }
    }

    /** Match the real sensor squashing. */
    static double getDutyCycle(Rotation2d r) {
        // This value can be negative.
        double rotations = r.getRotations();

        // This value is between 0 and 1.
        double rot01 = (rotations + 1) % 1;

        // Squash the value into the real sensor range.
        return rot01 * SENSOR_RANGE + AS5048RotaryPositionSensor.SENSOR_MIN;
    }

    /** For testing. */
    double output() {
        return m_sim.getOutput();
    }

}
