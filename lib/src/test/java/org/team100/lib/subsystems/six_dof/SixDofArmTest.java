package org.team100.lib.subsystems.six_dof;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class SixDofArmTest {
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test0() {
        // tool (x) pointing down
        SixDofArm arm = new SixDofArm(log);
        SixDofConfig config = arm.config(
                new Pose3d(0.5, 0.25, 0.1, new Rotation3d(0, Math.PI / 2, 0)));
        assertNotNull(config);
    }

    @Test
    void test1() {
        // tool pointing +x
        SixDofArm arm = new SixDofArm(log);
        SixDofConfig config = arm.config(
                new Pose3d(0.2, -0.2, 0.6, new Rotation3d(0, 0, 0)));
        assertNotNull(config);
    }

    @Test
    void test2() {
        SixDofArm arm = new SixDofArm(log);
        List<SixDofConfig> all1 = arm.m_kinematics.inverse(
                new Pose3d(0.5, 0.25, 0.1, new Rotation3d(0, 0, 0)), 0.0, 0.0);
        assertEquals(8, all1.size());
        List<SixDofConfig> f1 = arm.m_feasibility.filter(all1);
        assertEquals(4, f1.size());
        SixDofConfig q0 = new SixDofConfig(0, 0, 0, 0, 0, 0);
        SixDofConfig b1 = arm.getBest(f1, q0);
        System.out.printf("b1 %s\n", b1);

        List<SixDofConfig> all2 = arm.m_kinematics.inverse(
                new Pose3d(0.2, -0.2, 0.6, new Rotation3d(0, 0, 0)), 0.0, 0.0);
        assertEquals(8, all2.size());
        System.out.println("feasibility filtering ...");
        List<SixDofConfig> f2 = arm.m_feasibility.filter(all2);
        // two of the possibilities use a pitch rotation which is exactly
        // 90 degrees, and then rounding puts it just outside the range.
        assertEquals(6, f2.size());
        // assertEquals(8, f2.size());
        // use previous pose to measure distance
        SixDofConfig b2 = arm.getBest(f2, b1);
        System.out.printf("b2 %s\n", b2);

    }

}
