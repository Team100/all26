package org.team100.lib.geometry.r2;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents a direction in a 2d plane as a unit vector in R2.
 * 
 * The WPI class, Rotation2d, is used to represent both transformations (from
 * one direction to another) and directions themselves, using a "zero" direction
 * of (1,0).
 * 
 * It would be better to use distinct types to represent direction (this class)
 * differently from transformation (Rotation2d).
 * 
 * Possible parameterizations:
 * 
 * * angle relative to (1,0) (this is what Rotation2d does)
 * * sin and cos (also like Rotation2d)
 * * direction cosines with each axis (x, y), L2 norm = 1
 * 
 * The direction cosine is just a unit vector, so we'll do that.
 * 
 * @see https://en.wikipedia.org/wiki/Direction_cosine
 */
public class DirectionR2 {
    public final double x;
    public final double y;

    public DirectionR2(double px, double py) {
        double h = Math.sqrt(px * px + py * py);
        x = px / h;
        y = py / h;
    }

    public static DirectionR2 fromRotation(Rotation2d r) {
        return new DirectionR2(r.getCos(), r.getSin());
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof DirectionR2 other
                && Math.abs(other.x - x) < 1E-9
                && Math.abs(other.y - y) < 1E-9;
    }

}
