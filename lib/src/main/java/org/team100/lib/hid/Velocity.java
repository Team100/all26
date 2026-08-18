package org.team100.lib.hid;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This represents driver's velocity command, usually mapped to three axes in
 * the control, so the ranges are [-1,1]
 * 
 * TODO: add fields for acceleration.
 */
public record Velocity(double x, double y, double theta) {

    /**
     * Clip the translational velocity to the unit circle.
     * 
     * This means that there will be no stick response outside the circle, but the
     * inside will be unchanged.
     * 
     * The argument for clipping is that it leaves the response inside the circle
     * alone: with squashing, the diagonals are more sensitive.
     * 
     * The argument for squashing is that it preserves relative response in
     * the corners: with clipping, going full speed diagonally and then "slowing
     * down a little" will do nothing.
     * 
     * If you'd like to avoid clipping, then squash the input upstream, in the
     * control class.
     */
    public Velocity clip(double maxMagnitude) {
        double x = x();
        double y = y();
        double hyp = Math.hypot(x, y);
        if (hyp < 1e-12)
            return this;
        double clamped = Math.min(hyp, maxMagnitude);
        double ratio = clamped / hyp;
        return new Velocity(ratio * x, ratio * y, theta());
    }

    /**
     * Mecanum drive velocity envelope is a diamond shape: fastest in X, slower in
     * Y, even slower diagonally. This clips the stick input to the rhombus. Note
     * that the orientation of the rhombus depends on the robot's actual
     * orientation.
     */
    public Velocity diamond(double maxX, double maxY, Rotation2d poseAngle) {
        double x = x();
        double y = y();
        double hyp = Math.hypot(x, y);
        if (hyp < 1e-12)
            return this;
        Rotation2d fieldRelative = new Rotation2d(x, y);
        Rotation2d robotRelative = fieldRelative.minus(poseAngle);
        double r = 1 / (Math.abs(robotRelative.getCos() / maxX) + Math.abs(robotRelative.getSin() / maxY));
        double clamped = Math.min(hyp, r);
        double ratio = clamped / hyp;
        return new Velocity(ratio * x, ratio * y, theta());
    }


    public Velocity squashedDiamond(double maxX, double maxY, Rotation2d poseAngle) {
        double x = x();
        double y = y();
        double hyp = Math.hypot(x, y);
        if (hyp < 1e-12)
            return this;
        Rotation2d fieldRelative = new Rotation2d(x, y);
        Rotation2d robotRelative = fieldRelative.minus(poseAngle);
        // this is the maximum possible hyp
        double r = 1 / (Math.abs(robotRelative.getCos() / maxX) + Math.abs(robotRelative.getSin() / maxY));
        // assuming the max stick hyp is 1 (i.e. "round" stick response) then
        // r is also the scaling factor.
        return new Velocity(r * x, r * y, theta());
    }
}