package org.team100.frc2026.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;


    public class Obstacle {
       public final Rectangle2d rectangle;
       private final double clearance;

       public Obstacle(
        Pose2d center,
        double width,
        double height,
        double clearance) {
            rectangle = new Rectangle2d(
                center,
                (width + clearance),
                (height + clearance)
                );

        this.clearance = clearance;
    }
    public Rectangle2d getRectangle() {
        return rectangle;
    }

    public double getClearance() {
        return clearance;
    }

    public List<Translation2d> getCorners() {

        Pose2d center = rectangle.getCenter();

        double halfX = rectangle.getXWidth() / 2.0 ;
        double halfY = rectangle.getYWidth() / 2.0 ;

    List<Translation2d> corners = List.of(
        new Translation2d(-halfX, halfY),
        new Translation2d(halfX, halfY),
        new Translation2d(halfX, -halfY),
        new Translation2d(-halfX, -halfY)
    );

    return corners.stream()
            .map(corner ->
            center.getTranslation().plus(
                corner.rotateBy(center.getRotation())
            )
        )
        .toList();
    }
}