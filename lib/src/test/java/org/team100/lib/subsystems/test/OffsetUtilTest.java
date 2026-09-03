package org.team100.lib.subsystems.test;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;

public class OffsetUtilTest {
    @Test
    void test0() {
        // linear motion, no tangential velocity
        VelocitySE2 velocity = new VelocitySE2(1, 0, 0);
        Vector<N3> omega = OffsetUtil.omega(velocity);
        TestUtil.verify(VecBuilder.fill(0, 0, 0), omega);
        // from moving frame to center
        Translation2d m_offset = new Translation2d(0, 1);
        // from center to moving frame
        Translation2d r = m_offset.unaryMinus();
        Vector<N3> rvec = GeometryUtil.toVec3(r);
        TestUtil.verify(VecBuilder.fill(0, -1, 0), rvec);
        VelocitySE2 tangentialVelocity = OffsetUtil.tangentialVelocity(
                omega, rvec);
        TestUtil.verify(new VelocitySE2(0, 0, 0), tangentialVelocity);
    }

    @Test
    void test1() {
        // rotation
        VelocitySE2 velocity = new VelocitySE2(0, 0, 1);
        Vector<N3> omega = OffsetUtil.omega(velocity);
        TestUtil.verify(VecBuilder.fill(0, 0, 1), omega);
        // from moving frame to center
        Translation2d m_offset = new Translation2d(0, 1);
        // from center to moving frame
        Translation2d r = m_offset.unaryMinus();
        Vector<N3> rvec = GeometryUtil.toVec3(r);
        TestUtil.verify(VecBuilder.fill(0, -1, 0), rvec);
        VelocitySE2 tangentialVelocity = OffsetUtil.tangentialVelocity(
                omega, rvec);
        TestUtil.verify(new VelocitySE2(1, 0, 0), tangentialVelocity);
    }

    @Test
    void test2() {
        // minimal test of tangential velocity
        Vector<N3> omega = VecBuilder.fill(0, 0, 1);
        Vector<N3> rvec = VecBuilder.fill(0, -1, 0);
        VelocitySE2 tangentialVelocity = OffsetUtil.tangentialVelocity(
                omega, rvec);
        TestUtil.verify(new VelocitySE2(1, 0, 0), tangentialVelocity);
    }

    @Test
    void test3() {
        // minimal test of centripetal acceleration
        Vector<N3> omega = VecBuilder.fill(0, 0, 1);
        Vector<N3> rvec = VecBuilder.fill(0, -1, 0);
        AccelerationSE2 centripetalAccel = OffsetUtil.centripetalAcceleration(
                omega, rvec);
        TestUtil.verify(new AccelerationSE2(0, 1, 0), centripetalAccel);
    }

    @Test
    void test4() {
        // minimal test of centripetal acceleration
        Vector<N3> omega = VecBuilder.fill(0, 0, 2);
        Vector<N3> rvec = VecBuilder.fill(0, -1, 0);
        AccelerationSE2 centripetalAccel = OffsetUtil.centripetalAcceleration(
                omega, rvec);
        TestUtil.verify(new AccelerationSE2(0, 4, 0), centripetalAccel);
    }

    @Test
    void test5() {
        Vector<N3> reverseR = VecBuilder.fill(0, 0, 0);
        Vector<N3> cartesian = VecBuilder.fill(0, 0, 0);
        VelocitySE2 perpendicularOmega = OffsetUtil.omega(
                reverseR, cartesian);
        TestUtil.verify(new VelocitySE2(0, 0, 0), perpendicularOmega);
    }

}
