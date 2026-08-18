package org.team100.lib.localization;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Mirrors raspberry_pi BlipWithCorners.
 * 
 * Includes corner pixels (x,y).
 */
public class BlipWithCorners {
    private final long timestamp;
    private final int id;
    // these are laid out as primitives because python can't do an array.
    // https://github.com/robotpy/mostrobotpy/issues/272
    private final float x0;
    private final float y0;
    private final float x1;
    private final float y1;
    private final float x2;
    private final float y2;
    private final float x3;
    private final float y3;

    private final Transform3d pose;

    /**
     * @param timestamp server time microseconds
     * @param id        AprilTag id
     * @param corners   Four corners. Lower left first, then counter-clockwise.
     * @param pose      This uses the camera coordinate system, which has X to
     *                  the right, Y down, and Z forward.
     */
    public BlipWithCorners(long timestamp,
            int id,
            float[] corners,
            Transform3d pose) {
        this(
                timestamp, id,
                corners[0], corners[1],
                corners[2], corners[3],
                corners[4], corners[5],
                corners[6], corners[7],
                pose);
    }

    public BlipWithCorners(long timestamp, int id,
            float x0, float y0,
            float x1, float y1,
            float x2, float y2,
            float x3, float y3,
            Transform3d pose) {
        this.timestamp = timestamp;
        this.id = id;
        this.x0 = x0;
        this.y0 = y0;
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
        this.x3 = x3;
        this.y3 = y3;
        this.pose = pose;
    }

    public static BlipWithCorners fromXForward(
            long timestamp, int id, float[] corners, Transform3d pose) {
        return new BlipWithCorners(timestamp, id, corners, new Transform3d(
                GeometryUtil.xForwardToZForward(pose.getTranslation()),
                GeometryUtil.xForwardToZForward(pose.getRotation())));
    }

    /**
     * Server time in microseconds.
     * 
     * The timestamp of a blip is synchronized using Sync, so it is assumed to be
     * exactly in sync with the real server time; there are no additional offsets.
     */
    public long getTimestamp() {
        return timestamp;
    }

    /** Apriltag id number */
    public int getId() {
        return id;
    }

    /** Lower-left x. */
    public float getX0() {
        return x0;
    }

    /** Lower-left y. */
    public float getY0() {
        return y0;
    }

    /** Lower-right x. */
    public float getX1() {
        return x1;
    }

    /** Lower-right y. */
    public float getY1() {
        return y1;
    }

    /** Upper-right x. */
    public float getX2() {
        return x2;
    }

    /** Upper-right y. */
    public float getY2() {
        return y2;
    }

    /** Upper-left x. */
    public float getX3() {
        return x3;
    }

    /** Upper-left y. */
    public float getY3() {
        return y3;
    }

    /** Pixel corners of the tag. */
    public float[] getCorners() {
        return new float[] { x0, y0, x1, y1, x2, y2, x3, y3 };
    }

    /**
     * Raw tag transform produced by the camera.
     * 
     * The camera's coordinate system has X to the right, Y down, and Z forward, so
     * this Transform3d should not be used directly.
     */
    public Transform3d getRawPose() {
        return pose;
    }

    /**
     * Tag transform in X-forward coordinates.
     * 
     * Extract translation and rotation from z-forward blip and return the same
     * translation and rotation as an NWU x-forward transform. Package-private for
     * testing.
     */
    public Transform3d blipToTransform() {
        return new Transform3d(blipToTranslation(), blipToRotation());
    }

    @Override
    public String toString() {
        return "BlipWithCorners [timestamp = " + timestamp //
                + ", id=" + id //
                + ", pose=" + pose + "]";
    }

    public static final BlipWithCornersStruct struct = new BlipWithCornersStruct();

    /**
     * Extract the translation from a "z-forward" blip and return the same
     * translation expressed in our usual "x-forward" NWU translation.
     * It would be possible to also consume the blip rotation matrix, if it were
     * renormalized, but it's not very accurate, so we don't consume it.
     * Package-private for testing.
     */
    private Translation3d blipToTranslation() {
        return GeometryUtil.zForwardToXForward(pose.getTranslation());
    }

    /**
     * Extract the rotation from the "z forward" blip and return the same rotation
     * expressed in our usual "x forward" NWU coordinates. Package-private for
     * testing.
     */
    private Rotation3d blipToRotation() {
        return GeometryUtil.zForwardToXForward(pose.getRotation());
    }
}
