package org.team100.frc2025.field;

import org.team100.frc2025.field.FieldConstants2025.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstantsLuke {
    /** Staged coral ("lollipop") locations. */
    public enum CoralMark {
        LEFT(new Translation2d(1.219, 5.855)),
        CENTER(new Translation2d(1.219, 4.026)),
        RIGHT(new Translation2d(1.219, 2.197));

        public final Translation2d value;

        private CoralMark(Translation2d t) {
            this.value = t;
        }
    }

    private static final Translation2d REEF_CENTER = new Translation2d(4.489, 4.026);

    static Translation2d getScoringDestination(ReefPoint point, double radius) {
        Rotation2d sectorAngle = point.angle();
        Translation2d spoke = new Translation2d(radius, sectorAngle);
        // center of the face of the reef
        Translation2d center = REEF_CENTER.plus(spoke);
        Rotation2d newRotation = sectorAngle.rotateBy(Rotation2d.kCCW_90deg);

        // end-effector is not in the center of robot y, it's offset a little.
        double endEffectorOffset = -0.193;
        // Half the distance between the poles, i.e. offset from the center, in meters.
        double reefOffset = 0.1645; // 0.1524
        Translation2d offsetLeft = new Translation2d(endEffectorOffset - reefOffset, newRotation);
        Translation2d offsetRight = new Translation2d(endEffectorOffset + reefOffset, newRotation);
        Translation2d offsetCenter = new Translation2d(endEffectorOffset, newRotation);

        return switch (point) {
            // left side
            case A, C, E, G, I, K -> center.plus(offsetLeft);
            // right side
            case B, D, F, H, J, L -> center.plus(offsetRight);
            case AB, CD, EF, GH, IJ, KL -> center.plus(offsetCenter);
            case NONE -> center;
            default -> throw new IllegalArgumentException();
        };
    }

    public static Pose2d makeGoal(ScoringLevel level, ReefPoint point) {
        double radius = FieldConstants2025.getRadius(point, level);
        Translation2d destination = getScoringDestination(point, radius);
        Rotation2d heading = point.angle();
        return new Pose2d(destination, heading);
    }
}
