package org.team100.frc2026.auton;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;

public class ManeuverableGraph {

    public static Map<GraphAvoid, List<GraphAvoid>> build(
            Translation2d start,
            Translation2d goal,
            List<Obstacle> obstacles) {

        List<GraphAvoid> nodes = new ArrayList<>();
        nodes.add(new GraphAvoid(start));
        nodes.add(new GraphAvoid(goal));

        for (Obstacle obstacle : obstacles) {
            for (Translation2d corner : obstacle.getCorners()) {
                nodes.add(new GraphAvoid(corner));
            }
        }
        Map<GraphAvoid, List<GraphAvoid>> graph = new HashMap<>();
    return graph;
     }

    public static boolean intersectsObstacle(
            Translation2d a,
            Translation2d b,
            List<Obstacle> obstacles) {
        for (Obstacle obstacle : obstacles) {
            if (segmentIntersectsObstacle(
                    a,
                    b,
                    obstacle)) {
                return true;
            }
        }
        return false;
    }

    private static boolean segmentIntersectsObstacle(
            Translation2d a,
            Translation2d b,
            Obstacle obstacle) {
        List<Translation2d> c = obstacle.getCorners();

        return segmentIntersects(a, b, c.get(0), c.get(1)) ||
                segmentIntersects(a, b, c.get(1), c.get(2)) ||
                segmentIntersects(a, b, c.get(2), c.get(3)) ||
                segmentIntersects(a, b, c.get(3), c.get(0));
    }

    private static boolean segmentIntersects(
            Translation2d a,
            Translation2d b,
            Translation2d c,
            Translation2d d) {

        return ccw(a, c, d) != ccw(b, c, d) &&
                ccw(a, b, c) != ccw(a, b, d);
    }

    private static boolean ccw(
            Translation2d a,
            Translation2d b,
            Translation2d c) {
        return (c.getY() - a.getY()) * (b.getX() - a.getX()) > (b.getY() - a.getY()) * (c.getX() - a.getX());
    }
}