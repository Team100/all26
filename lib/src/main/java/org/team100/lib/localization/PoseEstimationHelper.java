package org.team100.lib.localization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Static methods used to interpret camera input.
 */
public class PoseEstimationHelper {
    /**
     * Invert the camera-to-tag, to get tag-to-camera.
     * Compose field-to-tag with tag-to-camera, to get field-to-camera.
     * Invert robot-to-camera, to get camera-to-robot.
     * Compose field-to-camera with camera-to-robot, to get field-to-robot.
     * 
     * This method trusts the tag rotation calculated by the camera.
     *
     * @param cameraInRobot Robot-to-camera, offset from Camera.java
     * @param tagInField    Field-to-tag, canonical pose from the JSON file
     * @param tagInCamera   Camera-to-tag, what the camera sees
     */
    public static Pose3d robotInField(
            Transform3d cameraInRobot,
            Pose3d tagInField,
            Transform3d tagInCamera) {
        // Camera in field frame.
        Pose3d cameraInField = cameraInField(tagInField, tagInCamera);
        // Robot in field frame.
        return robotInField(cameraInField, cameraInRobot);
    }

    //////////////////////////////
    //
    // Package-private below, don't use these.

    /**
     * Given the gyro rotation and the camera offset, return the camera absolute
     * rotation.
     */
    static Rotation3d cameraRotationInField(Transform3d cameraInRobot, Rotation3d gyroRotation) {
        return cameraInRobot.getRotation().rotateBy(gyroRotation);
    }

    /**
     * Because the camera's estimate of tag rotation isn't very accurate, this
     * synthesizes an estimate using the tag rotation in field frame (from json) and
     * the camera rotation in field frame (from gyro).
     */
    static Rotation3d tagRotationInCamera(
            Rotation3d tagRotationInField,
            Rotation3d cameraRotationInField) {
        return tagRotationInField.rotateBy(cameraRotationInField.unaryMinus());
    }

    /**
     * Invert the camera-to-tag, to get tag-to-camera.
     * Compose field-to-tag with tag-to-camera, to get field-to-camera.
     */
    static Pose3d cameraInField(Pose3d tagInField, Transform3d tagInCamera) {
        return tagInField.transformBy(tagInCamera.inverse());
    }

    /**
     * Invert robot-to-camera, to get camera-to-robot.
     * Compose field-to-camera with camera-to-robot, to get field-to-robot.
     */
    static Pose3d robotInField(Pose3d cameraInField, Transform3d cameraInRobot) {
        return cameraInField.transformBy(cameraInRobot.inverse());
    }
}
