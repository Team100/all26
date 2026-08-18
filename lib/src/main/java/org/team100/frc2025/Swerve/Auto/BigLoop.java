package org.team100.frc2025.Swerve.Auto;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
        Pose2d p1 = new Pose2d(
                p0.getX() + 2,
                p0.getY() - 2,
                p0.getRotation().plus(Rotation2d.kCCW_Pi_2));
        Pose2d p2 = new Pose2d(
                p0.getX() + 4,
                p0.getY(),
                p0.getRotation().plus(Rotation2d.k180deg));
        Pose2d p3 = new Pose2d(
                p0.getX() + 2,
                p0.getY() + 2,
                p0.getRotation().plus(Rotation2d.kCW_Pi_2));
        // note adjustment of scale and rotation "direction" (i.e. rate relative to
        // translation)
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        p0,
                        new DirectionSE2(1, 0, 0),
                        1),
                new WaypointSE2(
                        p1,
                        new DirectionSE2(1, 0, 0.5),
                        1.3),
                new WaypointSE2(
                        p2,
                        new DirectionSE2(0, 1, 0.5),
                        1.3),
                new WaypointSE2(
                        p3,
                        new DirectionSE2(-1, 0, 0.5),
                        1.3),
                new WaypointSE2(
                        p0,
                        new DirectionSE2(-1, 0, 0),
                        1)
        //
        );
        return m_planner.restToRest(waypoints);
    }
}
