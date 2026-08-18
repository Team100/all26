package org.team100.lib.trajectory.se2.constraint;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.spline.se2.SplineSE2;
import org.team100.lib.spline.se2.SplineSE2Factory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Verify that trajectory schedule generation yields a realistic profile.
 * 
 * https://docs.google.com/spreadsheets/d/16UUCCz-qcPz_YZMnsJnVkVO1KGp5zHCOVo7EoJct2nA/edit?gid=0#gid=0
 */
public class TrajectoryVelocityProfileTest implements Timeless {
    private static final boolean DEBUG = false;

    // A five-meter straight line.
    static WaypointSE2 w0 = WaypointSE2.irrotational(new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2);
    static WaypointSE2 w1 = WaypointSE2.irrotational(new Pose2d(2.5, 0, new Rotation2d(0)), 0, 1.2);
    static WaypointSE2 w2 = WaypointSE2.irrotational(new Pose2d(5, 0, new Rotation2d(0)), 0, 1.2);
    private static final List<WaypointSE2> waypoints = List.of(w0, w1, w2);
    static List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);

    private static PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.1, 0.1);
    private static PathSE2 path = pathFactory.get(splines);

    /**
     * Default max accel and velocity makes a very fast triangle profile.
     */
    @Test
    void testNoConstraint() {
        List<TimingConstraint> constraints = new ArrayList<TimingConstraint>();
        TrajectorySE2Factory u = new TrajectorySE2Factory(constraints);
        TrajectorySE2 traj = u.fromPath(path, 0, 0);
        if (DEBUG)
            traj.dump();
    }

    /**
     * This produces a trapezoid with the correct cruise (3.5) and accel/decel the
     * same (10)
     */
    @Test
    void testConstantConstraint() {
        // somewhat realistic numbers
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest();
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(1, 1, limits));
        TrajectorySE2Factory u = new TrajectorySE2Factory(constraints);
        TrajectorySE2 traj = u.fromPath(path, 0, 0);
        if (DEBUG)
            traj.dump();
    }

    /**
     * This produces the desired current-limited exponential shape for acceleration,
     * and faster decel at the end.
     * 
     * https://docs.google.com/spreadsheets/d/1sbB-zTBUjRRlWHaWXe-V1ZDhAZCFwItVVO1x3LmZ4B4/edit?gid=104506786#gid=104506786
     */
    @Test
    void testSwerveConstraint() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest();
        List<TimingConstraint> constraints = List.of(new SwerveDriveDynamicsConstraint(limits, 1, 1));
        TrajectorySE2Factory u = new TrajectorySE2Factory(constraints);
        TrajectorySE2 traj = u.fromPath(path, 0, 0);
        if (DEBUG)
            traj.dump();
    }

    /**
     * Realistic, cruses at 3.5 (which is right)
     * 
     * https://docs.google.com/spreadsheets/d/1sbB-zTBUjRRlWHaWXe-V1ZDhAZCFwItVVO1x3LmZ4B4/edit?gid=1802036642#gid=1802036642
     */
    @Test
    void testAuto() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest();
        TimingConstraintFactory timing = new TimingConstraintFactory(limits);
        List<TimingConstraint> constraints = timing.testAuto();
        TrajectorySE2Factory u = new TrajectorySE2Factory(constraints);
        TrajectorySE2 traj = u.fromPath(path, 0, 0);
        if (DEBUG)
            traj.dump();
    }
}
