package org.team100.lib.subsystems.swerve.module;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalEncoder;

/**
 * Uses simulated position sensors, must be used with clock control (e.g.
 * {@link Timeless}).
 */
public class SimulatedSwerveModule100 extends SwerveModule100 {
    private static final double DRIVE_GEAR_RATIO = 5.5;
    private static final double WHEEL_DIAMETER_M = 0.1;

    public static SimulatedSwerveModule100 get(LoggerFactory parent) {
        LinearMechanism drive = drive(parent.name("Drive"));
        RotaryMechanism steer = steer(parent.name("Turning"));
        return new SimulatedSwerveModule100(parent, drive, steer);
    }

    private static LinearMechanism drive(LoggerFactory parent) {
        SimulatedMotor motor = new SimulatedMotor(parent, 600);
        return new LinearMechanism(
                parent, motor, motor.encoder(), DRIVE_GEAR_RATIO, WHEEL_DIAMETER_M,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    /**
     * Uses simulated position sensors, must be used with clock control (e.g.
     * {@link Timeless}).
     */
    private static RotaryMechanism steer(LoggerFactory parent) {
        // simulated turning motor free speed is 20 rad/s
        SimulatedMotor turningMotor = new SimulatedMotor(parent, 600);
        IncrementalEncoder encoder = turningMotor.encoder();
        SimulatedRotaryPositionSensor turningSensor = new SimulatedRotaryPositionSensor(
                parent, encoder, 1);
        return new RotaryMechanism(
                parent, turningMotor, turningSensor, 1,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    private SimulatedSwerveModule100(
            LoggerFactory log, LinearMechanism drive, RotaryMechanism steer) {
        // primary is 2:1 so final is whatever is left.
        super(log, drive, steer, WHEEL_DIAMETER_M, DRIVE_GEAR_RATIO / 2);
    }
}
