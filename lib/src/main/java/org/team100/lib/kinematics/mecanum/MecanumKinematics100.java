package org.team100.lib.kinematics.mecanum;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.MecanumDriveKinematics;
import org.wpilib.math.kinematics.MecanumDriveWheelPositions;
import org.wpilib.math.kinematics.MecanumDriveWheelVelocities;

/**
 * Includes simple correction factors to account for wheel slip.
 * 
 * The slip varies depending on course -- more slip when moving sideways, less
 * slip when moving ahead.
 * 
 * Factor greater than one means the wheels will go faster than they would in
 * the zero-slip case, in order to make the actual velocity what is requested.
 */
public class MecanumKinematics100 {
    public record Slip(double kx, double ky, double ktheta) {
    }

    private final MecanumDriveKinematics m_kinematics;
    private final double m_kx;
    private final double m_ky;
    private final double m_ktheta;

    public MecanumKinematics100(
            Slip slip,
            Translation2d fl, Translation2d fr,
            Translation2d rl, Translation2d rr) {
        m_kinematics = new MecanumDriveKinematics(fl, fr, rl, rr);
        m_kx = slip.kx;
        m_ky = slip.ky;
        m_ktheta = slip.ktheta;
    }

    public MecanumDriveWheelVelocities toWheelVelocities(ChassisVelocities actual) {
        // Slipping wheels need to go faster than the actual speed.
        ChassisVelocities slipping = new ChassisVelocities(
                m_kx * actual.vx,
                m_ky * actual.vy,
                m_ktheta * actual.omega);
        return m_kinematics.toWheelVelocities(slipping);
    }

    public Twist2d toTwist2d(MecanumDriveWheelPositions start, MecanumDriveWheelPositions end) {
        Twist2d slipping = m_kinematics.toTwist2d(start, end);
        // Actual speed is slower than the slipping wheels would indicate.
        return new Twist2d(
                slipping.dx / m_kx,
                slipping.dy / m_ky,
                slipping.dtheta / m_ktheta);
    }

}
