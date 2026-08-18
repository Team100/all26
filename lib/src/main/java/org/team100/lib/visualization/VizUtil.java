package org.team100.lib.visualization;

import java.util.List;

import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Entry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class VizUtil {

    public static double[] poseToArray(Pose2d pose) {
        return new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees() };
    }

    public static double[] fromTrajectorySE2(TrajectorySE2 m_trajectory) {
        double[] arr = new double[m_trajectory.length() * 3];
        int ndx = 0;
        for (TrajectorySE2Entry p : m_trajectory.getPoints()) {
            WaypointSE2 pose = p.point().point().waypoint();
            arr[ndx + 0] = pose.pose().getTranslation().getX();
            arr[ndx + 1] = pose.pose().getTranslation().getY();
            arr[ndx + 2] = pose.pose().getRotation().getDegrees();
            ndx += 3;
        }
        return arr;
    }

    public static double[] fromWPITrajectory(Trajectory m_trajectory) {
        double[] arr = new double[m_trajectory.getStates().size() * 3];
        int ndx = 0;
        for (State p : m_trajectory.getStates()) {
            Pose2d pose = p.poseMeters;
            arr[ndx + 0] = pose.getTranslation().getX();
            arr[ndx + 1] = pose.getTranslation().getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        return arr;
    }

    public static double[] fromPoses(List<Pose2d> poses) {
        double[] arr = new double[poses.size() * 3];
        int ndx = 0;
        for (Pose2d pose : poses) {
            arr[ndx + 0] = pose.getTranslation().getX();
            arr[ndx + 1] = pose.getTranslation().getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        return arr;
    }
    
}
