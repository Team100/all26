package org.team100.lib.subsystems.rrr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.kinematics.rrr_se2.RRRFeasibility;
import org.team100.lib.kinematics.rrr_se2.RRRKinematicsPoE;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.WPITrapezoidProfileR1;
import org.team100.lib.state.ControlRRR;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.testing.TestUtil;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;

public class RRRProfileTest {
    @Test
    void test0() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(0.3, 0.3, 0.1);
        RRRFeasibility f = new RRRFeasibility(k,
                new RRRConfig(-Math.PI / 2, -Math.PI / 2, -Math.PI / 2),
                new RRRConfig(Math.PI / 2, Math.PI / 2, Math.PI / 2));
        ProfileR1 profile = new WPITrapezoidProfileR1(3, 6);
        RRRProfile p = new RRRProfile(k, f, profile);
        // current position is all zero, which is a singularity, and on the boundary.
        ControlRRR current = new ControlRRR(
                new RRRConfig(0, 0, 0),
                new RRRVelocity(0, 0, 0),
                new RRRAcceleration(0, 0, 0));
        TestUtil.verify(new Pose2d(0.7, 0, Rotation2d.kZero), k.forward(current.q()).p4());
        // goal is the same as the current position, motionless.
        ControlSE2 goal = new ControlSE2(
                new Pose2d(0.7, 0, Rotation2d.kZero),
                VelocitySE2.ZERO);
        ControlRRR setpoint = p.calculate(current, goal, 0.02);
        // so there's no change
        TestUtil.verify(new RRRConfig(0, 0, 0), setpoint.q());
        // Since xdot is zero, qdot is also zero
        TestUtil.verify(new RRRVelocity(0, 0, 0), setpoint.qdot());
        // since xdot and xddot are zero, qddot is zero
        TestUtil.verify(new RRRAcceleration(0, 0, 0), setpoint.qddot());
    }

    @Test
    void test1() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(0.3, 0.3, 0.1);
        RRRFeasibility f = new RRRFeasibility(k,
                new RRRConfig(-Math.PI / 2, -Math.PI / 2, -Math.PI / 2),
                new RRRConfig(Math.PI / 2, Math.PI / 2, Math.PI / 2));
        ProfileR1 profile = new WPITrapezoidProfileR1(3, 6);
        RRRProfile p = new RRRProfile(k, f, profile);
        // current position is all zero, which is a singularity, and on the boundary.
        ControlRRR current = new ControlRRR(
                new RRRConfig(0, 0, 0),
                new RRRVelocity(0, 0, 0),
                new RRRAcceleration(0, 0, 0));
        TestUtil.verify(new Pose2d(0.7, 0, Rotation2d.kZero), k.forward(current.q()).p4());
        // goal is the same as the current position, but wants to go toward the
        // boundary, which is impossible.
        ControlSE2 goal = new ControlSE2(
                new Pose2d(0.7, 0, Rotation2d.kZero),
                new VelocitySE2(1, 0, 0));
        ControlRRR setpoint = p.calculate(current, goal, 0.02);
        // the position is on the goal already
        TestUtil.verify(new RRRConfig(0, 0, 0), setpoint.q());
        // The jacobian inverse has all zeros for x velocity, so there's no qdot
        TestUtil.verify(new RRRVelocity(0, 0, 0), setpoint.qdot());
        TestUtil.verify(new RRRAcceleration(0, 0, 0), setpoint.qddot());
    }

    @Test
    void test2() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(0.3, 0.3, 0.1);
        RRRFeasibility f = new RRRFeasibility(k,
                new RRRConfig(-Math.PI / 2, -Math.PI / 2, -Math.PI / 2),
                new RRRConfig(Math.PI / 2, Math.PI / 2, Math.PI / 2));
        ProfileR1 profile = new WPITrapezoidProfileR1(3, 6);
        RRRProfile p = new RRRProfile(k, f, profile);
        // current position is all zero, which is a singularity, and on the boundary.
        ControlRRR current = new ControlRRR(
                new RRRConfig(0, 0, 0),
                new RRRVelocity(0, 0, 0),
                new RRRAcceleration(0, 0, 0));
        TestUtil.verify(new Pose2d(0.7, 0, Rotation2d.kZero), k.forward(current.q()).p4());
        // goal is the same as the current position, but wants to go tangentially along
        // the boundary, which is possible in an infinitesimal sense, but it does
        // violate the
        // boundary limit.
        ControlSE2 goal = new ControlSE2(
                new Pose2d(0.7, 0, Rotation2d.kZero),
                new VelocitySE2(0, 1, 0));
        ControlRRR setpoint = p.calculate(current, goal, 0.02);
        // the position is on the goal already
        TestUtil.verify(new RRRConfig(0, 0, 0), setpoint.q());
        // The velocity is not zero, because the arm *can* move tangentially to the
        // boundary,
        // in reality it will immediately cross the boundary with just Y motion.
        TestUtil.verify(new RRRVelocity(1.667, 0, -1.667), setpoint.qdot());
        TestUtil.verify(new RRRAcceleration(0, 0, 0), setpoint.qddot());
    }

    @Test
    void test3() {
        // move without a profile
        RRRKinematicsPoE k = new RRRKinematicsPoE(0.3, 0.3, 0.1);
        RRRFeasibility f = new RRRFeasibility(k,
                new RRRConfig(-Math.PI / 2, -Math.PI / 2, -Math.PI / 2),
                new RRRConfig(Math.PI / 2, Math.PI / 2, Math.PI / 2));
        ProfileR1 profile = new WPITrapezoidProfileR1(3, 6);
        RRRProfile p = new RRRProfile(k, f, profile);
        // all bent
        ControlRRR current = new ControlRRR(
                new RRRConfig(0.6, -1.2, 0.6),
                new RRRVelocity(0, 0, 0),
                new RRRAcceleration(0, 0, 0));
        TestUtil.verify(new Pose2d(0.595, 0, Rotation2d.kZero), k.forward(current.q()).p4());
        // rotate a bit -- less than the tolerance, so no profile here.
        ControlSE2 goal = new ControlSE2(
                new Pose2d(0.595, 0, new Rotation2d(0.01)),
                new VelocitySE2(0, 0, 0));
        ControlRRR setpoint = p.calculate(current, goal, 0.02);
        //
        TestUtil.verify(new RRRConfig(0.598, -1.201, 0.612), setpoint.q());
        TestUtil.verify(new Pose2d(0.595, 0, new Rotation2d(0.01)), k.forward(setpoint.q()).p4());
        //
        TestUtil.verify(new RRRVelocity(0.0, 0.0, 0.0), setpoint.qdot());
        TestUtil.verify(new RRRAcceleration(0.0, 0.0, 0.0), setpoint.qddot());
    }

    @Test
    void test4() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(0.3, 0.3, 0.1);
        RRRFeasibility f = new RRRFeasibility(k,
                new RRRConfig(-Math.PI / 2, -Math.PI / 2, -Math.PI / 2),
                new RRRConfig(Math.PI / 2, Math.PI / 2, Math.PI / 2));
        ProfileR1 profile = new WPITrapezoidProfileR1(1, 2);
        RRRProfile p = new RRRProfile(k, f, profile);
        // all bent
        ControlRRR current = new ControlRRR(
                new RRRConfig(0.6, -1.2, 0.6),
                new RRRVelocity(0, 0, 0),
                new RRRAcceleration(0, 0, 0));
        TestUtil.verify(new Pose2d(0.595, 0, Rotation2d.kZero), k.forward(current.q()).p4());
        ControlSE2 goal = new ControlSE2(
                new Pose2d(0.595, 0, new Rotation2d(0.0)),
                new VelocitySE2(0, 0, 0));
        ControlRRR setpoint = current;
        for (int i = 0; i < 200; ++i) {
            if (i > 10) {
                // goal moves suddenly
                goal = new ControlSE2(
                        new Pose2d(0.595, 0, new Rotation2d(1.5)),
                        new VelocitySE2(0, 0, 0));
            }
            setpoint = p.calculate(setpoint, goal, 0.02);
            System.out.printf("%d, %s, %s, %s\n",
                    i, setpoint.q(), setpoint.qdot(), StrUtil.poseStr(k.forward(setpoint.q()).p4()));
        }
    }

    @Test
    void test5() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(0.3, 0.3, 0.1);
        RRRFeasibility f = new RRRFeasibility(k,
                new RRRConfig(-Math.PI / 2, -Math.PI / 2, -Math.PI / 2),
                new RRRConfig(Math.PI / 2, Math.PI / 2, Math.PI / 2));
        ProfileR1 profile = new WPITrapezoidProfileR1(3, 6);
        RRRProfile p = new RRRProfile(k, f, profile);
        // all bent
        ControlRRR current = new ControlRRR(
                new RRRConfig(0.6, -1.2, 0.6),
                new RRRVelocity(0, 0, 0),
                new RRRAcceleration(0, 0, 0));
        TestUtil.verify(new Pose2d(0.595, 0, Rotation2d.kZero), k.forward(current.q()).p4());
        // rotate a lot: this should use the other posture.
        ControlSE2 goal = new ControlSE2(
                new Pose2d(0.595, 0, new Rotation2d(1.5)),
                new VelocitySE2(0, 0, 0));
        // use a very long dt in order to see the profile movement
        ControlRRR setpoint = p.calculate(current, goal, 2);
        // the setpoint should be using the other posture
        TestUtil.verify(new RRRConfig(-0.279, 0.221, 1.557), setpoint.q());
        // we're all the way to the goal.
        TestUtil.verify(new Pose2d(0.595, 0, new Rotation2d(1.5)), k.forward(setpoint.q()).p4());
        // motionless
        TestUtil.verify(new RRRVelocity(0, 0, 0), setpoint.qdot());
        TestUtil.verify(new RRRAcceleration(0, 0, 0), setpoint.qddot());
    }

    @Test
    void test6() {
        RRRConfig a = new RRRConfig(0, 0, 0);
        RRRConfig b = new RRRConfig(1, 0, 0);
        // q1 weight is 3, so this is sqrt(3)
        assertEquals(1.732, a.weightedDistance(b), 1e-3);
        assertEquals(1, a.euclideanDistance(b), 1e-3);
        Vector<N3> unit = RRRConfig.unit(a, b);
        // the unit vector is no longer scaled by the distance, it's just L2
        TestUtil.verify(VecBuilder.fill(1, 0, 0), unit);
        // multiply the unit vector by the distance to get b.
        Vector<N3> bVec = unit.times(1);
        TestUtil.verify(VecBuilder.fill(1, 0, 0), bVec);
    }

    @Test
    void test7() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(0.3, 0.3, 0.1);
        // bent to avoid the singularity.
        Pose2d x0 = new Pose2d(0.55, 0.1, Rotation2d.kCCW_90deg);
        List<RRRConfig> q0list = k.inverse(x0, null);
        assertEquals(2, q0list.size());
        TestUtil.verify(new RRRConfig(0.411, -0.822, 1.981), q0list.get(0));
        TestUtil.verify(new RRRConfig(-0.411, 0.822, 1.160), q0list.get(1));
        RRRConfig q0 = q0list.get(0);

        // -0.05 along x
        Pose2d x1 = new Pose2d(0.5, 0.1, Rotation2d.kCCW_90deg);
        List<RRRConfig> q1list = k.inverse(x1, null);
        assertEquals(2, q1list.size());
        TestUtil.verify(new RRRConfig(0.586, -1.171, 2.156), q1list.get(0));
        TestUtil.verify(new RRRConfig(-0.586, 1.171, 0.985), q1list.get(1));
        RRRConfig q1 = q1list.get(0);

        // average velocity along -x. Constant velocity should go from x0 to x1 in 1 sec
        VelocitySE2 xdot = new VelocitySE2(-0.05, 0, 0);
        Pose2d x = GeometryUtil.evolve(x0, xdot, new AccelerationSE2(0, 0, 0), 1);
        TestUtil.verify(x1, x);

        // what is this velocity at each q?
        RRRVelocity qdot0 = k.inverse(q0, xdot);
        TestUtil.verify(new RRRVelocity(0.208, -0.417, 0.208), qdot0);
        RRRVelocity qdot1 = k.inverse(q1, xdot);
        TestUtil.verify(new RRRVelocity(0.151, -0.301, 0.151), qdot1);

        // what's the average velocity?
        RRRVelocity qdotAv = RRRVelocity.fromVector(q1.toVector().minus(q0.toVector()));
        TestUtil.verify(new RRRVelocity(0.175, -0.350, 0.175), qdotAv);

        // where do you end up if you use these velocities for 1 sec?
        // they should be somewhere in the neighborhood of q1.get(0).
        RRRConfig q00 = q0.evolve(qdot0, new RRRAcceleration(0, 0, 0), 1);
        TestUtil.verify(new RRRConfig(0.620, -1.239, 2.190), q00);
        RRRConfig q01 = q0.evolve(qdot1, new RRRAcceleration(0, 0, 0), 1);
        TestUtil.verify(new RRRConfig(0.561, -1.124, 2.133), q01);

        Vector<N3> unit = RRRConfig.unit(q0, q1);
        double projectedQdot0 = qdot0.toVector().dot(unit);

        // this is the same as qdot0
        RRRVelocity qdot = RRRVelocity.fromVector(unit.times(projectedQdot0));
        TestUtil.verify(new RRRVelocity(0.208, -0.417, 0.208), qdot);

    }

}
