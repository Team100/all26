package org.team100.frc2025.field;

import org.team100.lib.config.ElevatorUtil.ScoringLevel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Positions on the field used as waypoints.
 */
public class FieldConstants2025 {
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

    /**
     * Scoring locations. Single letters are poles for coral, double letters are
     * between poles, for algae.
     */
    public enum ReefPoint {
        A, B, C, D, E, F, G, H, I, J, K, L, AB, CD, EF, GH, IJ, KL, NONE;

        /** Direction away from each reef face. */
        public Rotation2d angle() {
            return switch (this) {
                case A, B, AB -> Rotation2d.fromDegrees(180);
                case C, D, CD -> Rotation2d.fromDegrees(-120);
                case E, F, EF -> Rotation2d.fromDegrees(-60);
                case G, H, GH -> Rotation2d.fromDegrees(0);
                case I, J, IJ -> Rotation2d.fromDegrees(60);
                case K, L, KL -> Rotation2d.fromDegrees(120);
                case NONE -> Rotation2d.kZero;
                default -> throw new IllegalArgumentException();
            };
        }
    }

    public enum CoralStation {
        LEFT, RIGHT;

        public Pose2d pose() {
            return switch (this) {
                case LEFT -> new Pose2d(1.12, 7.14, Rotation2d.fromDegrees(-54)); // 1.2 7.0
                case RIGHT -> new Pose2d(1.12, 0.98, Rotation2d.fromDegrees(54));
                default -> throw new IllegalArgumentException("invalid station");
            };
        }
    }

    public static double getDistanceToReefCenter(Translation2d translation) {
        Translation2d target = REEF_CENTER.minus(translation);
        return target.getNorm();
    }

    public static Rotation2d angleToReefCenter(Translation2d t) {
        Translation2d target = REEF_CENTER.minus(t);
        Rotation2d targetAngle = target.getAngle();
        return targetAngle;
    }

    public static Rotation2d calculateDeltaSpline(Rotation2d originalRotation, Rotation2d deltaRotation, double scale) {
        Translation2d originalTranslation = new Translation2d(1, originalRotation);
        Translation2d deltaTranslation = new Translation2d(scale, deltaRotation);
        Translation2d newTranslation = originalTranslation.plus(deltaTranslation);
        return newTranslation.getAngle();
    }

    public static Pose2d makeGoal(ScoringLevel level, ReefPoint point) {
        double radius = getRadius(point, level);
        Translation2d destination = getScoringDestination(point, radius);
        Rotation2d heading = point.angle().rotateBy(Rotation2d.k180deg);
        return new Pose2d(destination, heading);
    }

    ///////////////////////////

    public static Translation2d getScoringDestination(ReefPoint point, double radius) {
        Rotation2d sectorAngle = point.angle();
        Translation2d spoke = new Translation2d(radius, sectorAngle);
        // center of the face of the reef
        Translation2d center = REEF_CENTER.plus(spoke);
        Rotation2d newRotation = sectorAngle.rotateBy(Rotation2d.kCCW_90deg);

        // end-effector is not in the center of robot y, it's offset a little.
        double endEffectorOffset = 0.193;
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

    /**
     * Distance to the center of the reef, for each scoring position and level.
     * 
     * These often need to be calibrated for specific reef setups, and to compensate
     * for any lingering inaccuracy in pose estimation.
     */
    public static double getRadius(ReefPoint letter, ScoringLevel level) {
        return switch (level) {
            case L1 -> switch (letter) {
                case A, B, C, D, E, F, G, H, I, J, K, L -> 3;
                case AB, CD, EF, GH, IJ, KL -> 1.2;
                case NONE -> 3.0;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case L2 -> switch (letter) {
                case A, B, C, D, E, F, G, H, I, J, K, L -> 1.4;
                case AB, CD, EF, GH, IJ, KL -> 1.2;
                case NONE -> 3.0;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case L3 -> switch (letter) {
                case A -> 1.4;
                case AB -> 1.2;
                case B -> 1.4;

                case C -> 1.4;
                case CD -> 1.2;
                case D -> 1.4;

                case E -> 1.4;
                case EF -> 1.2;
                case F -> 1.4;

                case G -> 1.4;
                case GH -> 1.2;
                case H -> 1.4;

                case I -> 1.4;
                case IJ -> 1.2;
                case J -> 1.4;

                case K -> 1.4;
                case KL -> 1.2;
                case L -> 1.4;
                case NONE -> 3.0;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case L4 -> switch (letter) {
                case A -> 1.415;
                case AB -> 1.2;
                case B -> 1.415;

                case C -> 1.415;
                case CD -> 1.2;
                case D -> 1.415;

                case E -> 1.415;
                case EF -> 1.2;
                case F -> 1.415;

                case G -> 1.415;
                case GH -> 1.2;
                case H -> 1.465;

                case I -> 1.415;
                case IJ -> 1.2;
                case J -> 1.415;

                case K -> 1.415;
                case KL -> 1.2;
                case L -> 1.415;
                case NONE -> 3.0;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case NONE -> 3.0;
            default -> throw new IllegalArgumentException("invalid level");
        };
    }

}
