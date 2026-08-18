package org.team100.lib.targeting;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Static methods for targeting.
 */
public class TargetUtil {
    /**
     * Absolute bearing from the robot to the target.
     * 
     * The bearing is only a valid shooting solution if both the robot and the
     * target are at rest!
     * 
     * If the robot and/or target is moving, then the shooting solution needs to
     * lead or lag the target.
     * 
     * @param robot  field-relative robot translation
     * @param target field-relative target translation
     * @return absolute bearing from robot to target
     */
    public static Rotation2d absoluteBearing(Translation2d robot, Translation2d target) {
        return target.minus(robot).getAngle();
    }

    /**
     * Absolute bearing close to the robot's current pose, might be outside
     * [-pi,pi].
     */
    public static double unwrappedAbsoluteBearing(Pose2d robot, Translation2d target) {
        Translation2d currentTranslation = robot.getTranslation();
        Rotation2d absoluteBearing = TargetUtil.absoluteBearing(currentTranslation, target);
        double yaw = robot.getRotation().getRadians();
        return Math100.getMinDistance(yaw, absoluteBearing.getRadians());
    }

    /**
     * Apparent motion of the target, NWU rad/s.
     * 
     * This is useful for feedforward, to track the apparent movement.
     * 
     * @param state  current robot state
     * @param target field-relative target position
     * @return apparent rotation of the target around the robot, rad/s
     */
    public static double targetMotion(ModelSE2 state, Translation2d target) {
        VelocitySE2 velocity = state.velocity();
        if (velocity.angle().isEmpty()) {
            // If there's no robot motion, there's no target motion.
            return 0;
        }
        Translation2d robot = state.pose().getTranslation();
        Translation2d robotToTarget = target.minus(robot);
        Rotation2d course = velocity.angle().get();
        double speedM_S = velocity.norm();
        double rangeM = robotToTarget.getNorm();
        Rotation2d absoluteBearing = absoluteBearing(robot, target);
        return targetMotion(course, speedM_S, rangeM, absoluteBearing);
    }

    /**
     * Apparent motion from course, speed, range, and bearing.
     * 
     * @param velocity        of the robot
     * @param rangeM          to the target
     * @param absoluteBearing to the target
     * @return apparent rotation of the target around the robot, rad/s
     */
    private static double targetMotion(
            Rotation2d course,
            double speedM_S,
            double rangeM,
            Rotation2d absoluteBearing) {
        Rotation2d bearingRelativeToCourse = absoluteBearing.minus(course);
        double perpendicularSpeedM_S = speedM_S * bearingRelativeToCourse.getSin();
        return perpendicularSpeedM_S / rangeM;
    }

    private TargetUtil() {
        //
    }
}
