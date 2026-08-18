package org.team100.frc2025.CalgamesArm;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.prr.PRRAcceleration;
import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.geometry.prr.PRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.kinematics.prr.PRRKinematics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Entry;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.TrajectorySE2Point;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.YawRateConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** How do the joints respond to trajectories? */
public class TrajectoryJointTest {
    private static final boolean DEBUG = false;
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    /**
     * How does the smooth cartesian trajectory work in configuration space?
     * 
     * Answer: it's fine, as long as the "reach up"/"reach down" discontinuity is
     * removed.
     * 
     * Acceleration is choppy in both cartesian and configuration, so maybe turn
     * that down a bit.
     * 
     * Charts here:
     * 
     * https://docs.google.com/spreadsheets/d/1yo5gU4NwVDUP8XaGb-7jNOtN6A_7cpX3DWRTfwgKym0/edit?gid=0#gid=0
     */
    @Test
    void homeToL4() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(1, 1),
                new YawRateConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(c);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner m_planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

        TrajectorySE2 t = m_planner.restToRest(List.of(
                WaypointSE2.irrotational(
                        new Pose2d(1, 0, new Rotation2d(0)), 0, 1.2),
                WaypointSE2.irrotational(
                        new Pose2d(1.9, 0.5, new Rotation2d(2.5)), 2, 1.2)));

        PRRKinematics k = new PRRKinematics(0.5, 0.3, PRRKinematics.Solver.ANALYTIC);
        if (DEBUG)
            System.out
                    .println(
                            "t, x, y, r, vx, vy, vr, ax, ay, ar, q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot");
        for (double tt = 0; tt < t.duration(); tt += 0.02) {
            TrajectorySE2Entry sample = t.sample(tt);
            TrajectorySE2Point point = sample.point();
            ControlSE2 m = point.control();
            Pose2d p = m.pose();
            VelocitySE2 v = m.velocity();
            AccelerationSE2 a = m.acceleration();
            PRRConfig q = k.inverse(p);
            PRRVelocity jv = k.inverse(m.model());
            PRRAcceleration ja = k.inverse(m);
            if (DEBUG) {
                System.out.printf(
                        "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        tt, p.getX(), p.getY(), p.getRotation().getRadians(), v.x(), v.y(), v.theta(), a.x(),
                        a.y(), a.theta(),
                        q.q1(), q.q2(), q.q3(),
                        jv.q1dot(), jv.q2dot(), jv.q3dot(),
                        ja.q1ddot(), ja.q2ddot(), ja.q3ddot());
            }
        }
    }

}
