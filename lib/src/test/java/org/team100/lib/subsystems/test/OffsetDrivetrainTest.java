package org.team100.lib.subsystems.test;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.se2.MockSubsystemSE2;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class OffsetDrivetrainTest {
    @Test
    void testNoOffset0() {
        MockSubsystemSE2 delegate = new MockSubsystemSE2(new StateSE2());
        Translation2d offset = new Translation2d(0, 0);
        OffsetDrivetrain od = new OffsetDrivetrain(delegate, offset);
        VelocityControlSE2 nextV = new VelocityControlSE2(0, 0, 0);
        od.set(nextV);
        TestUtil.verify(new VelocitySE2(0, 0, 0), delegate.m_setpoint.velocity());
    }

    @Test
    void testNoOffset1() {
        // with no offset, the delegate always gets the control
        MockSubsystemSE2 delegate = new MockSubsystemSE2(new StateSE2(new Pose2d(1, 2, Rotation2d.kCCW_Pi_2)));
        Translation2d offset = new Translation2d(0, 0);
        OffsetDrivetrain od = new OffsetDrivetrain(delegate, offset);
        VelocityControlSE2 nextV = new VelocityControlSE2(new VelocitySE2(1, 2, 3), new AccelerationSE2(4, 5, 6));
        od.set(nextV);
        TestUtil.verify(new VelocitySE2(1, 2, 3), delegate.m_setpoint.velocity());
        TestUtil.verify(new AccelerationSE2(4, 5, 6), delegate.m_setpoint.acceleration());
    }

    @Test
    void testLinear0() {
        // with offset, at origin, moving linearly
        MockSubsystemSE2 delegate = new MockSubsystemSE2(new StateSE2(new Pose2d(0, 0, Rotation2d.kZero)));
        Translation2d offset = new Translation2d(0, 1);
        OffsetDrivetrain od = new OffsetDrivetrain(delegate, offset);
        VelocityControlSE2 nextV = new VelocityControlSE2(new VelocitySE2(1, 2, 0), new AccelerationSE2(4, 5, 0));
        od.set(nextV);
        TestUtil.verify(new VelocitySE2(1, 2, 0), delegate.m_setpoint.velocity());
        TestUtil.verify(new AccelerationSE2(4, 5, 0), delegate.m_setpoint.acceleration());
    }

    @Test
    void testRotation0() {
        // offset +y, at origin, pure rotation
        MockSubsystemSE2 delegate = new MockSubsystemSE2(new StateSE2(new Pose2d(0, 0, Rotation2d.kZero)));
        Translation2d offset = new Translation2d(0, 1);
        OffsetDrivetrain od = new OffsetDrivetrain(delegate, offset);
        // pure rotation
        VelocityControlSE2 nextV = new VelocityControlSE2(new VelocitySE2(0, 0, 1), new AccelerationSE2(0, 0, 0));
        od.set(nextV);
        // rotating around (0, 1) means +x
        TestUtil.verify(new VelocitySE2(1, 0, 1), delegate.m_setpoint.velocity());
        // centripetal acceleration towards (0, 1)
        TestUtil.verify(new AccelerationSE2(0, 1, 0), delegate.m_setpoint.acceleration());
    }

    @Test
    void testBoth0() {
        // offset +y, at origin, both motiongs
        MockSubsystemSE2 delegate = new MockSubsystemSE2(new StateSE2(new Pose2d(0, 0, Rotation2d.kZero)));
        Translation2d offset = new Translation2d(0, 1);
        OffsetDrivetrain od = new OffsetDrivetrain(delegate, offset);
        // should leave the delegate without linear motion
        VelocityControlSE2 nextV = new VelocityControlSE2(new VelocitySE2(-1, 0, 1), new AccelerationSE2(0, 0, 2));
        od.set(nextV);
        // no linear motion
        TestUtil.verify(new VelocitySE2(0, 0, 1), delegate.m_setpoint.velocity());
        TestUtil.verify(new AccelerationSE2(0, 1, 2), delegate.m_setpoint.acceleration());
    }

}
