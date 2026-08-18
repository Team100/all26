package org.team100.lib.camera;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Represents all the cameras. Offsets that used to be here are
 * now in Offsets.java.
 * 
 * TODO: remove stale entries.
 */
public enum Camera {
    /** Camera bot rear facing, 10.1.0.24 */
    CAMERA_BACK("d44649628c20d4d4"),
    /** Camera bot front facing */
    CAMERA_FRONT("8ddb2ed6c49a9bce"),
    C("10000000a7c673d9"),
    /** Delta amp-placer */
    RIGHTAMP("10000000caeaae82"),
    /** Delta amp-placer */
    LEFTAMP("100000004e0a1fb9"),
    /** Delta intake */
    GAME_PIECE("1000000013c9c96c"),
    /** Right swerve */
    SWERVE_RIGHT("47403d5eafe002a9"),
    /** Left swerve */
    SWERVE_LEFT("8132c256f63bbb4e"),
    /** Funnel */
    FUNNEL("1e5acbaa5a7f9d10"),
    /** ALPHA Climber Camera Left */
    CLIMB_LEFT("82c4c3fe4f941e96"),
    /** ALPHA Climber Camera Right */
    CLIMB_RIGHT("364f07fb090a3bf7"),
    /** ALPHA Shooter Camera */
    SHOOTER("e47055a1bcbcead0"),
    //
    // For unit tests.
    //
    ORIGIN("origin"),
    TEST4("test4"),
    TEST5("test5"),
    TEST6("test6"),
    TEST7("test7"),
    TEST7A("test7a"),
    TEST8("test8"),
    TEST9("test9"),
    //
    // For simulation: four directions, a bit off the floor.
    //
    SIM0("sim0"),
    SIM1("sim1"),
    SIM2("sim2"),
    SIM3("sim3"),
    //
    // Maybe used in tests?
    //
    UNKNOWN(null);

    private static Map<String, Camera> cameras = new HashMap<>();
    static {
        for (Camera i : Camera.values()) {
            cameras.put(i.m_serialNumber, i);
        }
    }
    private String m_serialNumber;

    private Camera(String serialNumber) {
        m_serialNumber = serialNumber;
    }

    public static Camera get(String serialNumber) {
        if (cameras.containsKey(serialNumber))
            return cameras.get(serialNumber);
        // throw new IllegalArgumentException(
        // String.format("unknown camera serial number: \s", serialNumber));
        System.out.println("#############################################");
        System.out.println("###");
        System.out.printf("### Unknown camera serial number: %s\n", serialNumber);
        System.out.println("###");
        System.out.println("#############################################");
        return UNKNOWN;
    }

    public String getSerial() {
        return m_serialNumber;
    }

    /**
     * Use this to calibrate the cameras. Set the transform to identity, set a tag
     * in a known location ("robot to tag"), and enter what the camera thinks the
     * tag pose is -- this appears on glass as "camera to tag".
     */
    static Transform3d fromCalibration(Transform3d robotToTag, Transform3d cameraToTag) {
        return robotToTag.plus(cameraToTag.inverse());
    }
}
