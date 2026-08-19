package org.team100.frc2025.Swerve.Auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import org.team100.frc2025.field.FieldConstants2025;
import org.team100.frc2025.field.FieldConstants2025.CoralStation;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Function to supply a rest-to-rest trajectory from the given starting point to
 * the coral station.
 */
public class GoToCoralStation implements Function<Pose2d, TrajectorySE2> {
    private final double m_scale;
    private final CoralStation m_station;
    private final TrajectorySE2Planner m_planner;

    public GoToCoralStation(
            SwerveKinodynamics kinodynamics,
            CoralStation station,
            double scale) {
        m_station = station;
        m_scale = scale;
        List<TimingConstraint> constraints = new TimingConstraintFactory(kinodynamics).auto();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        m_planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
    }

    @Override
    public TrajectorySE2 apply(Pose2d currentPose) {
        Pose2d goal = m_station.pose();
        double scaleAdjust = switch (m_station) {
            case LEFT -> m_scale;
            case RIGHT -> -1.0 * m_scale;
            default -> throw new IllegalArgumentException("invalid station");
        };

        Translation2d currTranslation = currentPose.getTranslation();
        Rotation2d courseToGoal = goal.getTranslation().minus(currTranslation).getAngle();
        Rotation2d newInitialSpline = FieldConstants2025.calculateDeltaSpline(
                courseToGoal, courseToGoal.rotateBy(Rotation2d.fromDegrees(-90)), scaleAdjust);

        List<WaypointSE2> waypoints = new ArrayList<>();
        waypoints.add(new WaypointSE2(currentPose, DirectionSE2.irrotational(newInitialSpline), 1));
        waypoints.add(new WaypointSE2(goal, DirectionSE2.irrotational(courseToGoal), 1));

        return m_planner.restToRest(waypoints);
    }

}
