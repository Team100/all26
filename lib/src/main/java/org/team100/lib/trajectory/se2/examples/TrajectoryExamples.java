package org.team100.lib.trajectory.se2.examples;

import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.geometry.se2_universal_cover.WindingTransform2d;
import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;
import org.team100.lib.targeting.TargetUtil;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/** Examples that are mostly only useful for testing. */
public class TrajectoryExamples {

    private final TrajectorySE2Planner p;

    public TrajectoryExamples(TrajectorySE2Planner p) {
        this.p = p;
    }

    /** A square counterclockwise starting with +x. */
    public List<TrajectorySE2> square(Pose2d p0) {
        WindingPose2d p1 = new WindingPose2d(p0).plus(new WindingTransform2d(1, 0, 0));
        WindingPose2d p2 = new WindingPose2d(p0).plus(new WindingTransform2d(1, 1, 0));
        WindingPose2d p3 = new WindingPose2d(p0).plus(new WindingTransform2d(0, 1, 0));
        return List.of(
                restToRest(new WindingPose2d(p0), p1),
                restToRest(p1, p2),
                restToRest(p2, p3),
                restToRest(p3, new WindingPose2d(p0)));
    }

    /** Make a square that gets a reset starting point at each corner. */
    public List<Function<Pose2d, TrajectorySE2>> permissiveSquare() {
        return List.of(
                x -> restToRest(new WindingPose2d(x), new WindingPose2d(x.plus(new Transform2d(1, 0, Rotation2d.ZERO)))),
                x -> restToRest(new WindingPose2d(x), new WindingPose2d(x.plus(new Transform2d(0, 1, Rotation2d.ZERO)))),
                x -> restToRest(new WindingPose2d(x), new WindingPose2d(x.plus(new Transform2d(-1, 0, Rotation2d.ZERO)))),
                x -> restToRest(new WindingPose2d(x),
                        new WindingPose2d(x.plus(new Transform2d(0, -1, Rotation2d.ZERO)))));
    }

    /** From current to x+1 */
    public TrajectorySE2 line(Pose2d initial) {
        return restToRest(
                new WindingPose2d(initial),
                new WindingPose2d(initial).plus(new WindingTransform2d(1, 0, 0)));
    }

    /**
     * Produces straight lines from start to end, without rotation.
     * 
     * This is only useful for testing: in reality we always want rotation and
     * curves.
     */
    public TrajectorySE2 restToRest(WindingPose2d start, WindingPose2d end) {
        Optional<Rotation2d> courseToGoal = TargetUtil.absoluteBearing(start.getTranslation(), end.getTranslation());
        if (courseToGoal.isEmpty())
            return new TrajectorySE2();
        // direction towards goal without rotating
        DirectionSE2 direction = DirectionSE2.irrotational(courseToGoal.get());
        return p.restToRest(List.of(
                new WindingWaypointSE2(start, direction, 1),
                new WindingWaypointSE2(end, direction, 1)));
    }

}
