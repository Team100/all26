package org.team100.lib.subsystems.tank.commands;

import java.util.function.DoubleSupplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.subsystems.tank.TankDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

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
    private final ChassisSpeedsLogger m_logSpeed;

    private ChassisSpeeds m_speed;

    public TankManual(
            LoggerFactory parent,
            DoubleSupplier translation,
            DoubleSupplier rotation,
            double maxV,
            double maxOmega,
            TankDrive robotDrive) {
        LoggerFactory log = parent.type(this);
        m_logSpeed = log.chassisSpeedsLogger(Level.TRACE, "speed");
        m_translation = translation;
        m_rotation = rotation;
        m_maxV = maxV;
        m_maxOmega = maxOmega;
        m_drive = robotDrive;
        m_speed = new ChassisSpeeds();
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        Pair<ChassisSpeeds, ChassisAcceleration> setpoint = getSpeed();
        m_logSpeed.log(() -> setpoint.getFirst());
        m_drive.setVelocity(setpoint.getFirst(), setpoint.getSecond());
    }

    /** TODO: move to the control class */
    private Pair<ChassisSpeeds, ChassisAcceleration> getSpeed() {
        double translationM_S = MathUtil.applyDeadband(m_translation.getAsDouble(), 0.1) * m_maxV;
        double rotationRad_S = MathUtil.applyDeadband(m_rotation.getAsDouble(), 0.1) * m_maxOmega;
        ChassisSpeeds speed = m_drive.desaturate(translationM_S, rotationRad_S);
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
    private ChassisAcceleration accel(ChassisSpeeds speed) {
        ChassisAcceleration a = ChassisAcceleration.diff(
                m_speed, speed, TimedRobot100.LOOP_PERIOD_S);
        m_speed = speed;
        return a;
    }
}
