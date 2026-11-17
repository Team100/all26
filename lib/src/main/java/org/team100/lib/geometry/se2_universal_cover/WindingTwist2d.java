package org.team100.lib.geometry.se2_universal_cover;

import org.wpilib.math.geometry.Translation2d;

/** Just like Twist2d but without the exp() that wraps. */
public class WindingTwist2d {
    public double dx;
    public double dy;
    public double dtheta;

    public WindingTwist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    /** Does not wrap. */
    public WindingTransform2d exp() {
        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        double s;
        double c;
        if (Math.abs(dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }

        return new WindingTransform2d(
                new Translation2d(dx * s - dy * c, dx * c + dy * s), dtheta);
    }

}
