package org.team100.frc2025.Swerve.Auto;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.wpilib.math.geometry.Pose2d;

/** big looping trajectory for testing */
public class BigLoop implements Function<Pose2d, TrajectorySE2> {
    private final TrajectorySE2Planner m_planner;

    public BigLoop(SwerveKinodynamics kinodynamics) {
        List<TimingConstraint> constraints = new TimingConstraintFactory(kinodynamics).auto();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        m_planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
    }

    @Override
    public TrajectorySE2 apply(Pose2d p0) {
        // field-relative
        WindingPose2d p1 = new WindingPose2d(
                p0.getX() + 2,
                p0.getY() - 2,
                p0.getRotation().getRadians() + Math.PI / 2);
        WindingPose2d p2 = new WindingPose2d(
                p0.getX() + 4,
                p0.getY(),
                p0.getRotation().getRadians() + Math.PI);
        WindingPose2d p3 = new WindingPose2d(
                p0.getX() + 2,
                p0.getY() + 2,
                p0.getRotation().getRadians() - Math.PI / 2);
        // note adjustment of scale and rotation "direction" (i.e. rate relative to
        // translation)
        List<WindingWaypointSE2> waypoints = List.of(
                new WindingWaypointSE2(new WindingPose2d(p0), new DirectionSE2(1, 0, 0), 1),
                new WindingWaypointSE2(p1, new DirectionSE2(1, 0, 0.5), 1.3),
                new WindingWaypointSE2(p2, new DirectionSE2(0, 1, 0.5), 1.3),
                new WindingWaypointSE2(p3, new DirectionSE2(-1, 0, 0.5), 1.3),
                new WindingWaypointSE2(new WindingPose2d(p0), new DirectionSE2(-1, 0, 0), 1)
        //
        );
        return m_planner.restToRest(waypoints);
    }
}
