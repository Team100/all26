package org.team100.lib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.ChassisAcceleration;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisAccelerationTest {
    /**
     * Constant intrinsic speed, including rotation,
     * produces centrifugal force.
     */
    @Test
    void test0() {
        ChassisSpeeds v0 = new ChassisSpeeds(1, 0, 1);
        ChassisSpeeds v1 = new ChassisSpeeds(1, 0, 1);
        double dt = 0.02;
        ChassisAcceleration a = ChassisAcceleration.diff(v0, v1, dt);
        assertEquals(0, a.x(), 0.001);
        // Centrifugal accel normal to velocity.
        assertEquals(1, a.y(), 0.001);
        assertEquals(0, a.theta(), 0.001);
    }

}
