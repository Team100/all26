package org.team100.lib.subsystems.tank.commands;

import java.util.function.DoubleSupplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisVelocitiesLogger;
import org.team100.lib.subsystems.tank.TankDrive;
import org.wpilib.command2.Command;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Pair;

/**
 * Manual tank-drive control using a single joystick (if using an
 * xbox style control, this will be the right-hand stick).
 */
public class TankManual extends Command {
    private final DoubleSupplier m_translation;
    private final DoubleSupplier m_rotation;
    private final double m_maxV;
    private final double m_maxOmega;
    private final TankDrive m_drive;
    private final ChassisVelocitiesLogger m_logSpeed;

    private ChassisVelocities m_speed;

    public TankManual(
            LoggerFactory parent,
            DoubleSupplier translation,
            DoubleSupplier rotation,
            double maxV,
            double maxOmega,
            TankDrive robotDrive) {
        LoggerFactory log = parent.type(this);
        m_logSpeed = log.ChassisVelocitiesLogger(Level.TRACE, "speed");
        m_translation = translation;
        m_rotation = rotation;
        m_maxV = maxV;
        m_maxOmega = maxOmega;
        m_drive = robotDrive;
        m_speed = new ChassisVelocities();
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        Pair<ChassisVelocities, ChassisAcceleration> setpoint = getSpeed();
        m_logSpeed.log(() -> setpoint.getFirst());
        m_drive.setVelocity(setpoint.getFirst(), setpoint.getSecond());
    }

    /** TODO: move to the control class */
    private Pair<ChassisVelocities, ChassisAcceleration> getSpeed() {
        double translationM_S = MathUtil.applyDeadband(m_translation.getAsDouble(), 0.1) * m_maxV;
        double rotationRad_S = MathUtil.applyDeadband(m_rotation.getAsDouble(), 0.1) * m_maxOmega;
        ChassisVelocities speed = m_drive.desaturate(translationM_S, rotationRad_S);
        ChassisAcceleration accel = accel(speed);
        return new Pair<>(speed, accel);
    }

    /**
     * Compute acceleration using backwards finite difference
     * on chassis speed, using a constant DT.
     * 
     * This acceleration includes centrifugal force.
     * 
     * TODO: move to the control class
     */
    private ChassisAcceleration accel(ChassisVelocities speed) {
        ChassisAcceleration a = ChassisAcceleration.diff(
                m_speed, speed, TimedRobot100.LOOP_PERIOD_S);
        m_speed = speed;
        return a;
    }
}
