package org.team100.lib.localization.simulation;

import java.util.function.Function;

import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.uncertainty.AnisotropicNoiseSE2;
import org.team100.lib.uncertainty.AnisotropicNoiseSE2Sampler;
import org.team100.lib.uncertainty.OdometryNoise;
import org.wpilib.math.geometry.Twist2d;

public class AddAnisotropicOdometryNoise implements Function<SwerveModuleDeltas, Twist2d> {
    private final SwerveDriveKinematics100 m_k;
    private final AnisotropicNoiseSE2Sampler m_sampler;

    public AddAnisotropicOdometryNoise(SwerveDriveKinematics100 k) {
        m_k = k;
        m_sampler = new AnisotropicNoiseSE2Sampler();
    }

    /** apply forward kinematics and add noise */
    @Override
    public Twist2d apply(SwerveModuleDeltas d) {
        Twist2d t = m_k.forward(d);
        AnisotropicNoiseSE2 m_noise = OdometryNoise.get(m_k, d);
        return m_sampler.apply(m_noise, t);
    }

}
