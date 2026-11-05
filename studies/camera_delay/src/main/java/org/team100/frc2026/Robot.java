package org.team100.frc2026;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.network.NetworkUtil;
import org.team100.lib.network.RawTags;
import org.team100.lib.network.SimulatedCamera;
import org.team100.lib.network.Sync;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.wpi.SimulatedAS5048;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;
import org.team100.lib.util.Rotation2dInterpolator;
import org.team100.lib.util.TimeInterpolatableBuffer100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Turn a motor at moderate speed, observe its rotation in two ways, and compare
 * them.
 */
public class Robot extends TimedRobot100 {
    private static final boolean DEBUG = false;

    /**
     * Offset in turns, between the camera and the sensor.
     * 
     * This value is from the testboard, experimenting until the
     * "sensor minus camera" is zero at rest.
     */
    private static final double STATIC_OFFSET = 0.94922;

    /**
     * The AS5048 duty cycle sensor itself has a propagation delay of 100 us and a
     * sampling rate of 10 kHz, with a PWM output rate of 1 kHz.
     * 
     * https://www.mouser.com/datasheet/2/588/AS5048_DS000298_4-00-1100510.pdf
     * 
     * The RoboRIO is counting the duty cycle duration, and needs to get to the end
     * of the pulse (1ms) in order to make a measurement.
     * 
     * So the measurement represents at least 1ms in the past, plus the 100 us
     * delay.
     */
    private static final double SENSOR_DELAY_S = 0.0011;
    /**
     * Maximum rotational speed.
     * 
     * Note: with positional control very high speeds don't work
     * because of the discretization of the goal.
     */
    private static final double MOTOR_SPEED_RAD_S = 50.0;

    private final BareMotor m_motor;
    private final AS5048RotaryPositionSensor m_sensor;
    private final RawTags m_rawTags;

    /** Motor measurement history, used for simulation.. */
    private final TimeInterpolatableBuffer100<Rotation2d> m_motorBuffer;
    /** Duty-cycle sensor measurement history. */
    private final TimeInterpolatableBuffer100<Rotation2d> m_sensorBuffer;
    /** Camera measurement history. */
    private final TimeInterpolatableBuffer100<Rotation2d> m_cameraBuffer;
    /** Clock sync with the Pi. */
    private final Sync m_sync;

    private final Rotation2dLogger m_logMotor;
    private final Rotation2dLogger m_logSensor;
    private final Rotation2dLogger m_logCamera;
    private final Rotation2dLogger m_logSensorMinusCamera;
    private final Rotation2dLogger m_logMotorMinusCamera;
    private final Rotation2dLogger m_logMotorMinusSensor;

    private final SimulatedAS5048 m_simSensor;
    private final SimulatedCamera m_simCamera;

    private final XboxController m_controller;

    /** Motor position setpoint. */
    private double m_positionRad;

    public Robot() {
        LoggerFactory log = Logging.instance().rootLogger;
        m_sync = new Sync(NetworkTableInstance.getDefault());

        m_controller = new XboxController(0);
        m_motor = getMotor(log);
        m_sensor = new AS5048RotaryPositionSensor(
                log,
                new RoboRioChannel(0),
                STATIC_OFFSET, // offset in turns
                EncoderDrive.INVERSE);

        m_motorBuffer = newBuffer();
        m_sensorBuffer = newBuffer();
        m_cameraBuffer = newBuffer();

        // Receive camera measurements. Update the buffer with the roll component, and
        // accept the supplied timestamp as exact.
        m_rawTags = new RawTags(log, new Roll(log, this::putBlip));

        m_logMotor = log.rotation2dLogger(Level.TRACE, "motor");
        m_logSensor = log.rotation2dLogger(Level.TRACE, "sensor");
        m_logCamera = log.rotation2dLogger(Level.TRACE, "camera");
        m_logSensorMinusCamera = log.rotation2dLogger(Level.TRACE, "sensor minus camera");
        m_logMotorMinusCamera = log.rotation2dLogger(Level.TRACE, "motor minus camera");
        m_logMotorMinusSensor = log.rotation2dLogger(Level.TRACE, "motor minus sensor");

        m_simSensor = getSensorSim();
        m_simCamera = getCameraSim();

        waitForDutyCycleBug();
    }

    private TimeInterpolatableBuffer100<Rotation2d> newBuffer() {
        Rotation2dInterpolator interpolator = new Rotation2dInterpolator();
        double historyDuration = 2;
        int initialTime = 0;
        Rotation2d initialValue = Rotation2d.kZero;
        return new TimeInterpolatableBuffer100<>(
                interpolator, historyDuration, initialTime, initialValue);
    }

    @Override
    public void robotPeriodic() {
        // Update the clock.
        Takt.update();

        // Reply to sync requests.
        m_sync.run();

        if (DEBUG)
            System.out.printf("takt %f\n", Takt.actual());

        // Update measurements, and update the simulated motor output to match the
        // input.
        Cache.refresh();

        // Read the motor and update the motor buffer.
        measureMotor();

        // Run the simulations if present.
        runSims();

        // Update the real AS5048 to read the value written by the simulated one.
        Cache.refresh();

        // Read the sensor and update the sensor buffer.
        measureSensor();

        // Read the camera and update the camera buffer.
        m_rawTags.update();

        // Sample the buffers from 1 sec ago and log.
        double oneSecAgo = Takt.actual() - 1.0;
        logDiffs(sampleMotor(oneSecAgo), sampleSensor(oneSecAgo), sampleCamera(oneSecAgo));

        m_motor.periodic();
        m_sensor.periodic();
        // Poll for logs after all the actuation is done
        LogPoller.log();
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void teleopInit() {
        m_positionRad = 0;
        m_motor.setUnwrappedEncoderPositionRad(m_positionRad);
        for (ConnectionInfo ci : NetworkTableInstance.getDefault().getConnections()) {
            System.out.printf("*** CONNECTION %s\n", NetworkUtil.ciString(ci));
        }
    }

    @Override
    public void teleopPeriodic() {
        double motorSpeedRadS = MOTOR_SPEED_RAD_S * (m_controller.getLeftTriggerAxis());
        // Use a fixed DT to avoid injecting noise into the motor command.
        m_positionRad += motorSpeedRadS * TimedRobot100.LOOP_PERIOD_S;
        m_motor.setUnwrappedPosition(m_positionRad, motorSpeedRadS, 0);
    }

    @Override
    public void teleopExit() {
        m_motor.stop();
    }

    private void putBlip(Rotation2d r, double t) {
        m_cameraBuffer.put(t, r);
    }

    private BareMotor getMotor(LoggerFactory log) {
        if (RobotBase.isReal()) {
            return new NeoVortexCANSparkMotor(
                    log,
                    new TotalCurrentLog(log),
                    new CanId(9),
                    NeutralMode100.COAST,
                    MotorPhase.FORWARD,
                    new CurrentLimit(20, 20),
                    new Friction(log, 0.15, 0.06, 0.0, 0.5),
                    PIDConstants.makePositionPID(log, 0.5), 0, 0);
        } else {
            return new SimulatedBareMotor(log, 600);
        }
    }

    private Rotation2d motorForSim(double x) {
        return wrap(m_motorBuffer.get(x));
    }

    private SimulatedCamera getCameraSim() {
        if (RobotBase.isSimulation()) {
            return new SimulatedCamera(this::motorForSim);
        } else {
            return null;
        }
    }

    private SimulatedAS5048 getSensorSim() {
        if (RobotBase.isSimulation()) {
            return new SimulatedAS5048(this::motorForSim, m_sensor);
        } else {
            return null;
        }
    }

    /** Run the simulated sensor and camera if present. */
    private void runSims() {
        if (m_simSensor != null) {
            m_simSensor.run();
        }
        if (m_simCamera != null) {
            m_simCamera.run();
        }
    }

    /** Sample the motor history. */
    private Rotation2d sampleMotor(double t) {
        Rotation2d sample = wrap(m_motorBuffer.get(t));
        if (DEBUG)
            System.out.printf("lagged motor %s\n", sample);
        m_logMotor.log(() -> sample);
        return sample;
    }

    /** Sample the sensor history. */
    private Rotation2d sampleSensor(double t) {
        Rotation2d sample = wrap(m_sensorBuffer.get(t));
        if (DEBUG)
            System.out.printf("lagged sensor %s\n", sample);
        m_logSensor.log(() -> sample);
        return sample;
    }

    /** Sample the camera history. */
    private Rotation2d sampleCamera(double t) {
        Rotation2d sample = wrap(m_cameraBuffer.get(t));
        m_logCamera.log(() -> sample);
        return sample;
    }

    /** Log the differences between the measurements. */
    private void logDiffs(Rotation2d motor, Rotation2d sensor, Rotation2d camera) {
        m_logSensorMinusCamera.log(() -> sensor.minus(camera));
        m_logMotorMinusCamera.log(() -> motor.minus(camera));
        m_logMotorMinusSensor.log(() -> motor.minus(sensor));
    }

    /** Record the motor position in the motor buffer. */
    private void measureMotor() {
        // The motor state is updated in the cache refresh.
        Rotation2d measurement = new Rotation2d(MathUtil.angleModulus(m_motor.getUnwrappedPositionRad()));
        m_motorBuffer.put(Takt.actual(), measurement);
        if (DEBUG)
            System.out.printf("motor value %s\n", measurement);
    }

    /** Record the sensor position in the sensor buffer. */
    private void measureSensor() {
        // For simulation, the value here is the same one that was just written by the
        // sim.
        Rotation2d measurement = new Rotation2d(m_sensor.getWrappedPositionRad());
        m_sensorBuffer.put(Takt.actual() - SENSOR_DELAY_S, measurement);
        if (DEBUG)
            System.out.printf("sensor value %s\n", measurement);
    }

    /** Wrap x within [-pi, pi]. */
    private Rotation2d wrap(Rotation2d x) {
        // The extra addition wraps the result.
        return x.plus(Rotation2d.kZero);
    }

    /**
     * RoboRIO duty-cycle inputs don't work correctly until a few seconds after
     * startup.
     */
    private static void waitForDutyCycleBug() {
        try {
            if (RobotBase.isReal())
                Thread.sleep(3000);
        } catch (InterruptedException e) {
        }
    }
}
