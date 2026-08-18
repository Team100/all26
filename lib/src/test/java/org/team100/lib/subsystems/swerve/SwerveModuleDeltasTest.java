package org.team100.lib.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class SwerveModuleDeltasTest {
        @Test
    void testRoundTripModuleDeltas() {
        SwerveDriveKinematics100 m_kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        {
            // straight diagonal path
            Twist2d t = new Twist2d(1, 1, 0);
            SwerveModuleDeltas p = m_kinematics.inverse(t);
            Twist2d t2 = m_kinematics.forward(p);
            assertEquals(t, t2);
        }
        {
            // turning and moving
            Twist2d t = new Twist2d(1, 1, 1);
            SwerveModuleDeltas p = m_kinematics.inverse(t);
            Twist2d t2 = m_kinematics.forward(p);
            assertEquals(t, t2);
        }
        {
            // turning and moving really fast
            Twist2d t = new Twist2d(10, 10, 10);
            SwerveModuleDeltas p = m_kinematics.inverse(t);
            Twist2d t2 = m_kinematics.forward(p);
            assertEquals(t, t2);
        }
        Random random = new Random(0);
        for (int i = 0; i < 500; ++i) {
            // inverse always works
            Twist2d t = new Twist2d(
                    random.nextDouble(),
                    random.nextDouble(),
                    random.nextDouble());
            SwerveModuleDeltas p = m_kinematics.inverse(t);
            Twist2d t2 = m_kinematics.forward(p);
            assertEquals(t, t2);
        }
    }
}
