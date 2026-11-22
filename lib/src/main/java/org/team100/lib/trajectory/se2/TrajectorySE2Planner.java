package org.team100.lib.trajectory.se2;

import java.util.List;

import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;
import org.team100.lib.path.se2.PathSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.spline.se2.SplineSE2;
import org.team100.lib.spline.se2.SplineSE2Factory;

/**
 * Creates a trajectory in three steps:
 * 
 * 1. create a spline
 * 2. create points along the spline so that the secants between the points are
 * within the spline sample tolerance, and the points are close enough together
 * 3. assign timestamps to each step
 * 
 * This used to support moving start and end states but we never used it, so
 * it's gone.
 */
public class TrajectorySE2Planner {
    private static final boolean DEBUG = false;

    private final PathSE2Factory m_pathFactory;
    private final TrajectorySE2Factory m_trajectoryFactory;

    public TrajectorySE2Planner(PathSE2Factory pathFactory, TrajectorySE2Factory trajectoryFactory) {
        m_pathFactory = pathFactory;
        m_trajectoryFactory = trajectoryFactory;
    }

    /**
     * Makes a trajectory through the supplied waypoints, starting and ending
     * motionless.
     */
    public TrajectorySE2 restToRest(List<WindingWaypointSE2> waypoints) {
        return generateTrajectory(waypoints, 0.0, 0.0);
    }

    /////////////////////////////////////////////////////////////////////////////////////
    ///
    /// DANGER ZONE
    ///
    /// Only use this if you know what you're doing.

    /**
     * Makes a trajectory through the supplied waypoints, with start and end
     * velocities.
     */
    public TrajectorySE2 generateTrajectory(
            List<WindingWaypointSE2> waypoints, double start_vel, double end_vel) {
        try {
            // Create splines from waypoints.
            List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);
            // Create a path from splines.
            PathSE2 path = m_pathFactory.get(splines);
            if (DEBUG)
                System.out.printf("PATH\n%s\n", path);
            // Generate the timed trajectory.
            TrajectorySE2 result = m_trajectoryFactory.fromPath(path, start_vel, end_vel);
            if (DEBUG)
                System.out.printf("TRAJECTORY\n%s\n", result);
            return result;
        } catch (IllegalArgumentException e) {
            // catches various kinds of malformed input, returns a no-op.
            // this should never actually happen.
            System.out.println("WARNING: Bad trajectory input!!");
            // print the stack trace if you want to know who is calling
            e.printStackTrace();
            return new TrajectorySE2();
        }
    }
}
