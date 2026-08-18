package org.team100.lib.trajectory.se2.examples;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.targeting.TargetUtil;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/** Examples that are mostly only useful for testing. */
public class TrajectoryExamples {

    private final TrajectorySE2Planner p;

    public TrajectoryExamples(TrajectorySE2Planner p) {
        this.p = p;
    }

    /** A square counterclockwise starting with +x. */
    public List<TrajectorySE2> square(Pose2d p0) {
        Pose2d p1 = p0.plus(new Transform2d(1, 0, Rotation2d.kZero));
        Pose2d p2 = p0.plus(new Transform2d(1, 1, Rotation2d.kZero));
        Pose2d p3 = p0.plus(new Transform2d(0, 1, Rotation2d.kZero));
        return List.of(
                restToRest(p0, p1),
                restToRest(p1, p2),
                restToRest(p2, p3),
                restToRest(p3, p0));
    }

    /** Make a square that gets a reset starting point at each corner. */
    public List<Function<Pose2d, TrajectorySE2>> permissiveSquare() {
        return List.of(
                x -> restToRest(x, x.plus(new Transform2d(1, 0, Rotation2d.kZero))),
                x -> restToRest(x, x.plus(new Transform2d(0, 1, Rotation2d.kZero))),
                x -> restToRest(x, x.plus(new Transform2d(-1, 0, Rotation2d.kZero))),
                x -> restToRest(x, x.plus(new Transform2d(0, -1, Rotation2d.kZero))));
    }

    /** From current to x+1 */
    public TrajectorySE2 line(Pose2d initial) {
        return restToRest(
                initial,
                initial.plus(new Transform2d(1, 0, Rotation2d.kZero)));
    }

    /**
     * Produces straight lines from start to end, without rotation.
     * 
     * This is only useful for testing: in reality we always want rotation and curves.
     */
    public TrajectorySE2 restToRest(Pose2d start, Pose2d end) {
        Rotation2d courseToGoal = TargetUtil.absoluteBearing(start.getTranslation(), end.getTranslation());
        // direction towards goal without rotating
        DirectionSE2 direction = DirectionSE2.irrotational(courseToGoal);
        return p.restToRest(List.of(
                new WaypointSE2(start, direction, 1),
                new WaypointSE2(end, direction, 1)));
    }

    

}
