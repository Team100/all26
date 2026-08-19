package org.team100.lib.kinematics.prr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.function.Function;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.prr.PRRAcceleration;
import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.geometry.prr.PRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.optimization.NumericalJacobian100;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Entry;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.examples.TrajectoryExamples;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Nat;

public class PRRKinematicsTest {
    private static final boolean DEBUG = false;
    // one micrometer tolerance since all the math here is exact
    private static final double FINE = 0.000001;
    private static final double COARSE = 0.001;

    @Test
    void testArmHeightComp() {
        PRRKinematics k = new PRRKinematics(5, 1, PRRKinematics.Solver.ANALYTIC);
        Translation2d wristPosition = new Translation2d(3, 3);
        double h = k.armX(wristPosition);
        if (DEBUG)
            System.out.println(wristPosition.getY());
        assertEquals(4, h);
    }

    @Test
    void testForward0() {
        PRRKinematics k = new PRRKinematics(0.3, 0.1, PRRKinematics.Solver.ANALYTIC);
        PRRConfig c = new PRRConfig(1, Math.toRadians(60), Math.toRadians(0));
        Pose2d p = k.forward(c);
        // 60 degrees so x is half the total length
        assertEquals(1.2, p.getX(), FINE);
        // 30/60/90 triangle, this side is sqrt(3)/2
        assertEquals(0.4 * Math.sqrt(3) / 2, p.getY(), FINE);
        // should be the same as the input
        assertEquals(Math.toRadians(60), p.getRotation().getRadians(), FINE);
    }

    @Test
    void testForward1() {
        PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);
        // one meter high, zero shoulder (so to the right along x), zero wrist (also
        // along x)
        PRRConfig c = new PRRConfig(1, 0, 0);
        Pose2d p = k.forward(c);
        // should be the height plus the sum of the link lengths
        assertEquals(4.0, p.getX(), FINE);
        // straight up
        assertEquals(0.0, p.getY(), FINE);
        // relative angle should be zero
        assertEquals(Math.toRadians(0), p.getRotation().getRadians(), FINE);
    }

    @Test
    void testInverse0() {
        // should be straight up
        PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);
        Pose2d p = new Pose2d(4, 0, Rotation2d.kZero);
        PRRConfig c = k.inverse(p);
        // pose at 4, total is 3 long, so shoulder at 1
        assertEquals(1, c.q1(), FINE);
        assertEquals(0, c.q2(), FINE);
        assertEquals(0, c.q3(), FINE);
    }

    @Test
    void testInverseDownArm45Triangle() {
        // built for a 45 45 90 triangle for
        PRRKinematics k = new PRRKinematics((2 * Math.sqrt(2)), 1, PRRKinematics.Solver.ANALYTIC);
        Pose2d p = new Pose2d(0.1, 3, Rotation2d.kCCW_90deg);
        PRRConfig c = k.inverse(p);

        assertEquals(-1.9, c.q1(), 0.001);
        assertEquals(Math.toRadians(45), c.q2(), 0.001);
        assertEquals(Math.toRadians(45), c.q3(), 0.001);
    }

    @Test
    void testInverseDownArm() {
        PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);
        Pose2d p = new Pose2d(0.1, 2, Rotation2d.kCCW_90deg);
        PRRConfig c = k.inverse(p);

        assertEquals(0.1 - Math.sqrt(3), c.q1(), 0.001);
        assertEquals(Math.toRadians(30), c.q2(), 0.001);
        assertEquals(Math.toRadians(60), c.q3(), 0.001);
    }

    @Test
    void testInverse1() {
        // arm up, wrist to the side
        PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);
        Pose2d p = new Pose2d(3, 1, Rotation2d.kCCW_90deg);
        PRRConfig c = k.inverse(p);
        // arm length is 2, wrist location is at 3
        assertEquals(1, c.q1(), FINE);
        assertEquals(0, c.q2(), FINE);
        assertEquals(Math.PI / 2, c.q3(), FINE);
    }

    @Test
    void testInverse2() {
        // arm to the side, wrist down
        PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);
        Pose2d p = new Pose2d(0, 2, Rotation2d.k180deg);
        PRRConfig c = k.inverse(p);
        assertEquals(1, c.q1(), FINE);
        assertEquals(Math.PI / 2, c.q2(), FINE);
        assertEquals(Math.PI / 2, c.q3(), FINE);
    }

    @Test
    void testRoundTripInverseFirst() {
        PRRKinematics k = new PRRKinematics(0.3, 0.1, PRRKinematics.Solver.ANALYTIC);
        Pose2d p = new Pose2d(1.178, 0.207, new Rotation2d(Math.toRadians(55)));

        PRRConfig c2 = k.inverse(p);
        Pose2d p2 = k.forward(c2);

        assertEquals(p.getX(), p2.getX(), FINE);
        assertEquals(p.getY(), p2.getY(), FINE);
        assertEquals(p.getRotation().getRadians(), p2.getRotation().getRadians(), FINE);

    }

    @Test
    void testRoundTripForwardFirst() {
        PRRKinematics k = new PRRKinematics(0.3, 0.1, PRRKinematics.Solver.ANALYTIC);
        PRRConfig c = new PRRConfig(1, Math.toRadians(60), Math.toRadians(60));

        Pose2d p2 = k.forward(c);
        assertEquals(1.1, p2.getX(), FINE);
        assertEquals(0.3 * Math.sqrt(3) / 2 + 0.1 * Math.sqrt(3) / 2, p2.getY(), FINE);
        assertEquals(Math.toRadians(120), p2.getRotation().getRadians(), FINE);

        PRRConfig c2 = k.inverse(p2);
        assertEquals(c.q1(), c2.q1(), FINE);
        assertEquals(c.q2(), c2.q2(), FINE);
        assertEquals(c.q3(), c2.q3(), FINE);
    }

    @Test
    void testForward() {
        final PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);
        PRRConfig q = new PRRConfig(0, 0, 0);
        // extended and motionless
        PRRVelocity jv = new PRRVelocity(0, 0, 0);
        VelocitySE2 v = k.forward(q, jv);
        assertEquals(0, v.x(), COARSE);
        assertEquals(0, v.y(), COARSE);
        assertEquals(0, v.theta(), COARSE);

        // +shoulder => +y and +theta
        jv = new PRRVelocity(0, 1, 0);
        v = k.forward(q, jv);
        assertEquals(0, v.x(), COARSE);
        assertEquals(3, v.y(), COARSE);
        assertEquals(1, v.theta(), COARSE);

        // +wrist => +y and +theta
        jv = new PRRVelocity(0, 0, 1);
        v = k.forward(q, jv);
        assertEquals(0, v.x(), COARSE);
        assertEquals(1, v.y(), COARSE);
        assertEquals(1, v.theta(), COARSE);

        // bent at shoulder, +shoulder => -x, +theta
        q = new PRRConfig(0, Math.PI / 2, 0);
        jv = new PRRVelocity(0, 1, 0);
        v = k.forward(q, jv);
        assertEquals(-3, v.x(), COARSE);
        assertEquals(0, v.y(), COARSE);
        assertEquals(1, v.theta(), COARSE);

        // bent at shoulder and wrist, +wrist => -y, +theta
        q = new PRRConfig(0, Math.PI / 2, Math.PI / 2);
        jv = new PRRVelocity(0, 0, 1);
        v = k.forward(q, jv);
        assertEquals(0, v.x(), COARSE);
        assertEquals(-1, v.y(), COARSE);
        assertEquals(1, v.theta(), COARSE);
    }

    @Test
    void testInverse() {
        final PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);

        PRRConfig c = new PRRConfig(1, 0, 0);
        Pose2d p = k.forward(c);

        // some example velocities
        // zero velocity
        ModelSE2 v = new ModelSE2(p);

        PRRVelocity jv = k.inverse(v);
        assertEquals(0, jv.q1dot(), COARSE);
        assertEquals(0, jv.q2dot(), COARSE);
        assertEquals(0, jv.q3dot(), COARSE);

        // +x
        v = new ModelSE2(p, new VelocitySE2(1, 0, 0));
        jv = k.inverse(v);
        assertEquals(1, jv.q1dot(), COARSE);
        assertEquals(0, jv.q2dot(), COARSE);
        assertEquals(0, jv.q3dot(), COARSE);

        // +y
        v = new ModelSE2(p, new VelocitySE2(0, 1, 0));
        jv = k.inverse(v);
        assertEquals(0, jv.q1dot(), COARSE);
        assertEquals(0.5, jv.q2dot(), COARSE);
        assertEquals(-0.5, jv.q3dot(), COARSE);

        // +theta
        v = new ModelSE2(p, new VelocitySE2(0, 0, 1));
        jv = k.inverse(v);
        assertEquals(0, jv.q1dot(), COARSE);
        assertEquals(-0.5, jv.q2dot(), COARSE);
        assertEquals(1.5, jv.q3dot(), COARSE);
    }

    @Test
    void testForwardA() {
        final PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);
        PRRConfig q = new PRRConfig(0, 0, 0);
        // extended, motionless
        PRRVelocity qdot = new PRRVelocity(0, 0, 0);
        PRRAcceleration qddot = new PRRAcceleration(0, 0, 0);
        AccelerationSE2 a = k.forward(q, qdot, qddot);
        assertEquals(0, a.x(), COARSE);
        assertEquals(0, a.y(), COARSE);
        assertEquals(0, a.theta(), COARSE);

        // +elevator => +x
        q = new PRRConfig(0, 0, 0);
        qdot = new PRRVelocity(0, 0, 0);
        qddot = new PRRAcceleration(1, 0, 0);
        a = k.forward(q, qdot, qddot);
        assertEquals(1, a.x(), COARSE);
        assertEquals(0, a.y(), COARSE);
        assertEquals(0, a.theta(), COARSE);

        // +shoulder => +y, +theta
        q = new PRRConfig(0, 0, 0);
        qdot = new PRRVelocity(0, 0, 0);
        qddot = new PRRAcceleration(0, 1, 0);
        a = k.forward(q, qdot, qddot);
        assertEquals(0, a.x(), COARSE);
        assertEquals(3, a.y(), COARSE);
        assertEquals(1, a.theta(), COARSE);

        // +wrist => +y, +theta
        q = new PRRConfig(0, 0, 0);
        qdot = new PRRVelocity(0, 0, 0);
        qddot = new PRRAcceleration(0, 0, 1);
        a = k.forward(q, qdot, qddot);
        assertEquals(0, a.x(), COARSE);
        assertEquals(1, a.y(), COARSE);
        assertEquals(1, a.theta(), COARSE);

        // shoulder bent, +shoulder => -x, +theta
        q = new PRRConfig(0, Math.PI / 2, 0);
        qdot = new PRRVelocity(0, 0, 0);
        qddot = new PRRAcceleration(0, 1, 0);
        a = k.forward(q, qdot, qddot);
        assertEquals(-3, a.x(), COARSE);
        assertEquals(0, a.y(), COARSE);
        assertEquals(1, a.theta(), COARSE);

        // shoulder and wrist bent, +wrist => -y, +theta
        q = new PRRConfig(0, Math.PI / 2, Math.PI / 2);
        qdot = new PRRVelocity(0, 0, 0);
        qddot = new PRRAcceleration(0, 0, 1);
        a = k.forward(q, qdot, qddot);
        assertEquals(0, a.x(), COARSE);
        assertEquals(-1, a.y(), COARSE);
        assertEquals(1, a.theta(), COARSE);
    }

    @Test
    void testInverseA() {
        final PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);

        // extended, motionless
        PRRConfig c = new PRRConfig(0, 0, 0);
        Pose2d p = k.forward(c);
        VelocitySE2 v = new VelocitySE2(0, 0, 0);
        ControlSE2 m = new ControlSE2(p, v, new AccelerationSE2(0, 0, 0));
        PRRAcceleration ja = k.inverse(m);
        assertEquals(0, ja.q1ddot(), COARSE);
        assertEquals(0, ja.q2ddot(), COARSE);
        assertEquals(0, ja.q3ddot(), COARSE);

        // +x => +elevator
        c = new PRRConfig(0, 0, 0);
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(1, 0, 0));
        ja = k.inverse(m);
        assertEquals(1, ja.q1ddot(), COARSE);
        assertEquals(0, ja.q2ddot(), COARSE);
        assertEquals(0, ja.q3ddot(), COARSE);

        // +y => +shoulder and -wrist (because zero theta)
        c = new PRRConfig(0, 0, 0);
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(0, 1, 0));
        ja = k.inverse(m);
        assertEquals(0, ja.q1ddot(), COARSE);
        assertEquals(0.5, ja.q2ddot(), COARSE);
        assertEquals(-0.5, ja.q3ddot(), COARSE);

        // +theta => -shoulder, +wrist (because no translation)
        c = new PRRConfig(0, 0, 0);
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(0, 0, 1));
        ja = k.inverse(m);
        assertEquals(0, ja.q1ddot(), COARSE);
        assertEquals(-0.5, ja.q2ddot(), COARSE);
        assertEquals(1.5, ja.q3ddot(), COARSE);

        // bent shoulder, +x => +elevator only
        // using 45 deg because of singularity at 90
        c = new PRRConfig(0, Math.PI / 4, 0);
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(1, 0, 0));
        ja = k.inverse(m);
        assertEquals(1, ja.q1ddot(), COARSE);
        assertEquals(0, ja.q2ddot(), COARSE);
        assertEquals(0, ja.q3ddot(), COARSE);

        // bent shoulder, +y => +elevator, +shoulder, -wrist (because no theta)
        c = new PRRConfig(0, Math.PI / 4, 0);
        // using 45 deg because of singularity at 90
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(0, 1, 0));
        ja = k.inverse(m);
        assertEquals(1, ja.q1ddot(), COARSE);
        assertEquals(0.707, ja.q2ddot(), COARSE);
        assertEquals(-0.707, ja.q3ddot(), COARSE);

        // bent shoulder and wrist, +y => +elevator, +shoulder, -wrist
        c = new PRRConfig(0, Math.PI / 4, Math.PI / 4);
        // using 45 deg because of singularity at 90
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(0, 1, 0));
        ja = k.inverse(m);
        assertEquals(1, ja.q1ddot(), COARSE);
        assertEquals(0.707, ja.q2ddot(), COARSE);
        assertEquals(-0.707, ja.q3ddot(), COARSE);
    }

    @Test
    void testTrajectory() {
        final PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);

        List<TimingConstraint> constraints = List.of(new ConstantConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        Pose2d start = new Pose2d(1, -1, Rotation2d.kZero);
        Pose2d end = new Pose2d(2, 1, Rotation2d.k180deg);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 t = ex.restToRest(start, end);
        double d = t.duration();
        double dt = d / 20;
        for (double time = 0; time < d; time += dt) {
            TrajectorySE2Entry tp = t.sample(time);
            ModelSE2 sm = ModelSE2.fromMovingPathPointSE2(tp.point().point(), tp.point().velocity());
            Pose2d p = sm.pose();
            VelocitySE2 v = sm.velocity();
            PRRConfig c = k.inverse(p);
            PRRVelocity jv = k.inverse(sm);
            if (DEBUG)
                System.out.printf(
                        "s (%5.2f) pose(%5.2f %5.2f %5.2f) conf(%5.2f %5.2f %5.2f) tv(%5.2f %5.2f %5.2f) jv(%5.2f %5.2f %5.2f)\n",
                        time,
                        p.getX(), p.getY(), p.getRotation().getRadians(),
                        c.q1(), c.q2(), c.q3(),
                        v.x(), v.y(), v.theta(),
                        jv.q1dot(), jv.q2dot(), jv.q3dot());
        }
    }

    @Test
    void test0() {
        final PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);
        Function<Vector<N3>, Vector<N3>> f = //
                q -> GeometryUtil.toVec(k.forward(PRRConfig.fromVector(q)));

        Matrix<N3, N3> j = NumericalJacobian100.numericalJacobian(
                Nat.N3(),
                Nat.N3(),
                f,
                new PRRConfig(1, 0, 0).toVector());
        if (DEBUG)
            System.out.println(j);

        // dx/dh should be 1 in all positions
        assertEquals(1, j.get(0, 0), COARSE);
        // dx/dshoulder, zero in this position
        assertEquals(0, j.get(0, 1), 0.002);
        // dx/dwrist, zero in this position
        assertEquals(0, j.get(0, 2), COARSE);

        // dy/dh should be zero in all positions
        assertEquals(0, j.get(1, 0), COARSE);
        // dy/dshoulder depends on the radius (3 here)
        assertEquals(3, j.get(1, 1), COARSE);
        // dy/dwrist depends on the radius (1 here)
        assertEquals(1, j.get(1, 2), COARSE);

        // dr/dh should always be zero
        assertEquals(0, j.get(2, 0), COARSE);
        // dr/dshoulder = 1
        assertEquals(1, j.get(2, 1), COARSE);
        // dr/dwrist = 1
        assertEquals(1, j.get(2, 2), COARSE);

        // invertible
        assertEquals(2, j.det(), COARSE);

        Matrix<N3, N3> jinv = j.inv();
        // dh/dx
        assertEquals(1, jinv.get(0, 0), COARSE);
        // dh/dy
        assertEquals(0, jinv.get(0, 1), COARSE);
        // dh/dr
        assertEquals(0, jinv.get(0, 2), COARSE);

        // dshoulder/dx
        assertEquals(0, jinv.get(1, 0), COARSE);
        // dshoulder/dy, +y -> +shoulder.
        assertEquals(0.5, jinv.get(1, 1), COARSE);
        // dshoulder/dr, to rotate only, the shoulder compensates the other way
        assertEquals(-0.5, jinv.get(1, 2), COARSE);

        // dwrist/dx
        assertEquals(0, jinv.get(2, 0), COARSE);
        // dwrist/dy, +y -> shoulder moves, wrist compensates the other way
        assertEquals(-0.5, jinv.get(2, 1), COARSE);
        // dwrist/dr extra to counter the compensation
        assertEquals(1.5, jinv.get(2, 2), COARSE);

        // some example velocities
        // zero velocity
        Vector<N3> v = VecBuilder.fill(0, 0, 0);
        Vector<N3> m = new Vector<>(jinv.times(v));
        assertEquals(0, m.get(0), COARSE);
        assertEquals(0, m.get(1), COARSE);
        assertEquals(0, m.get(2), COARSE);

        // +x
        v = VecBuilder.fill(1, 0, 0);
        m = new Vector<>(jinv.times(v));
        assertEquals(1, m.get(0), COARSE);
        assertEquals(0, m.get(1), COARSE);
        assertEquals(0, m.get(2), COARSE);

        // +y
        v = VecBuilder.fill(0, 1, 0);
        m = new Vector<>(jinv.times(v));
        assertEquals(0, m.get(0), COARSE);
        assertEquals(0.5, m.get(1), COARSE);
        assertEquals(-0.5, m.get(2), COARSE);

        // +theta
        v = VecBuilder.fill(0, 0, 1);
        m = new Vector<>(jinv.times(v));
        assertEquals(0, m.get(0), COARSE);
        assertEquals(-0.5, m.get(1), COARSE);
        assertEquals(1.5, m.get(2), COARSE);
    }

    @Test
    void test05() {
        final PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.NUMERIC);

        PRRConfig c = new PRRConfig(1, 0, 0);
        Pose2d p = k.forward(c);

        // some example velocities
        // zero velocity
        ModelSE2 v = new ModelSE2(p);

        PRRVelocity jv = k.inverse(v);
        assertEquals(0, jv.q1dot(), COARSE);
        assertEquals(0, jv.q2dot(), COARSE);
        assertEquals(0, jv.q3dot(), COARSE);

        // +x
        v = new ModelSE2(p, new VelocitySE2(1, 0, 0));
        jv = k.inverse(v);
        assertEquals(1, jv.q1dot(), COARSE);
        assertEquals(0, jv.q2dot(), COARSE);
        assertEquals(0, jv.q3dot(), COARSE);

        // +y
        v = new ModelSE2(p, new VelocitySE2(0, 1, 0));
        jv = k.inverse(v);
        assertEquals(0, jv.q1dot(), COARSE);
        assertEquals(0.5, jv.q2dot(), COARSE);
        assertEquals(-0.5, jv.q3dot(), COARSE);

        // +theta
        v = new ModelSE2(p, new VelocitySE2(0, 0, 1));
        jv = k.inverse(v);
        assertEquals(0, jv.q1dot(), COARSE);
        assertEquals(-0.5, jv.q2dot(), COARSE);
        assertEquals(1.5, jv.q3dot(), COARSE);
    }

    @Test
    void testDet() {
        // this prints a table of jacobian determinants ("scale") to show the
        // singularities at the edges.
        final PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);

        // forward jacobian goes from config to pose
        Function<Vector<N3>, Vector<N3>> f = //
                q -> GeometryUtil.toVec(k.forward(PRRConfig.fromVector(q)));

        for (double x = 4; x >= 0; x -= 0.2) {
            for (double y = -2; y <= 2; y += 0.2) {
                // for now, end-effector rotation is always zero (i.e. facing up)
                Pose2d p = new Pose2d(x, y, Rotation2d.kZero);
                Matrix<N3, N3> j = NumericalJacobian100.numericalJacobian(
                        Nat.N3(),
                        Nat.N3(),
                        f,
                        k.inverse(p).toVector());
                double det = j.det();
                if (DEBUG)
                    System.out.printf("%8.2f", det);
            }
            if (DEBUG)
                System.out.println();
        }
    }

    @Test
    void testPath() {
        // a very simple path showing pose, config, and velocities.
        //
        final PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.ANALYTIC);
        Function<Vector<N3>, Vector<N3>> f = //
                q -> GeometryUtil.toVec(k.forward(PRRConfig.fromVector(q)));

        Pose2d start = new Pose2d(1, -1, Rotation2d.kZero);
        Pose2d end = new Pose2d(2, 1, Rotation2d.k180deg);
        double stepSize = 0.1;
        Pose2d prev = start;
        for (double s = 0; s <= 1.0; s += stepSize) {
            Pose2d p = GeometryUtil.interpolate(start, end, s);
            Vector<N3> tv = VecBuilder.fill(
                    p.getX() - prev.getX(),
                    p.getY() - prev.getY(),
                    p.getRotation().minus(prev.getRotation()).getRadians());
            PRRConfig c = k.inverse(p);
            Matrix<N3, N3> j = NumericalJacobian100.numericalJacobian(
                    Nat.N3(),
                    Nat.N3(),
                    f,
                    c.toVector());
            Matrix<N3, N3> jinv = j.inv();
            Vector<N3> cv = new Vector<>(jinv.times(tv));
            if (DEBUG)
                System.out.printf(
                        "s (%5.2f) pose(%5.2f %5.2f %5.2f) conf(%5.2f %5.2f %5.2f) tv(%5.2f %5.2f %5.2f) cv(%5.2f %5.2f %5.2f)\n",
                        s,
                        p.getX(), p.getY(), p.getRotation().getRadians(),
                        c.q1(), c.q2(), c.q3(),
                        tv.get(0), tv.get(1), tv.get(2),
                        cv.get(0), cv.get(1), cv.get(2));
            prev = p;
        }
    }

    @Test
    void testTrajectoryA() {
        final PRRKinematics k = new PRRKinematics(2, 1, PRRKinematics.Solver.NUMERIC);

        List<TimingConstraint> constraints = List.of(new ConstantConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        Pose2d start = new Pose2d(1, -1, Rotation2d.kZero);
        Pose2d end = new Pose2d(2, 1, Rotation2d.k180deg);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 t = ex.restToRest(start, end);
        double d = t.duration();
        double dt = d / 20;
        for (double time = 0; time < d; time += dt) {
            TrajectorySE2Entry tp = t.sample(time);
            ModelSE2 sm = ModelSE2.fromMovingPathPointSE2(tp.point().point(), tp.point().velocity());
            Pose2d p = sm.pose();
            VelocitySE2 v = sm.velocity();
            PRRConfig c = k.inverse(p);
            PRRVelocity jv = k.inverse(sm);
            if (DEBUG)
                System.out.printf(
                        "s (%5.2f) pose(%5.2f %5.2f %5.2f) conf(%5.2f %5.2f %5.2f) tv(%5.2f %5.2f %5.2f) jv(%5.2f %5.2f %5.2f)\n",
                        time,
                        p.getX(), p.getY(), p.getRotation().getRadians(),
                        c.q1(), c.q2(), c.q3(),
                        v.x(), v.y(), v.theta(),
                        jv.q1dot(), jv.q2dot(), jv.q3dot());
        }
    }

}
