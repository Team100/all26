package org.team100.lib.uncertainty;

import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose2d;

/** Container for a pose and its uncertainty. */
public class NoisyPose2d {
    private final Pose2d m_pose;
    private final IsotropicNoiseSE2 m_noise;

    public NoisyPose2d(Pose2d pose, IsotropicNoiseSE2 noise) {
        m_pose = pose;
        m_noise = noise;
    }

    public Pose2d pose() {
        return m_pose;
    }

    public IsotropicNoiseSE2 noise() {
        return m_noise;
    }

    @Override
    public String toString() {
        return "NoisyPose2d [m_pose=" + StrUtil.pose2Str(m_pose) + ", m_noise=" + m_noise + "]";
    }

}
