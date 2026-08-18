package org.team100.lib.spline.se2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N2;

public class SplineSE2Util {
    /**
     * Position of offset for parameter, s.
     * 
     * The offset is a fixed length behind the toolpoint.
     */
    public static Vector<N2> offsetR(SplineSE2 spline, double l, double s) {
        Pose2d pose = spline.pose(s);
        // negative length here because the offset is "behind" the toolpoint.
        Pose2d p2 = pose.transformBy(new Transform2d(-l, 0, new Rotation2d()));
        return VecBuilder.fill(p2.getX(), p2.getY());
    }

    /**
     * Derivative of offset with respect to parameter, s.
     * 
     * Since the offset length is fixed, the only velocity is tangential, from the
     * heading rate.
     */
    public static Vector<N2> offsetRprime(SplineSE2 spline, double l, double s) {
        Pose2d pose = spline.pose(s);
        Rotation2d radial = pose.getRotation().plus(Rotation2d.k180deg);
        Rotation2d tangential = radial.plus(Rotation2d.kCCW_Pi_2);
        // dtheta/ds
        double headingRate = spline.dheading(s);
        // dx/ds = dtheta/ds * x/theta
        double tangentialVelocity = l * headingRate;
        return VecBuilder.fill(tangentialVelocity * tangential.getCos(), tangentialVelocity * tangential.getSin());
    }

    /**
     * Second derivative of offset with respect to parameter, s.
     * 
     * The offset point acceleration has two components: tangential and centripetal.
     */
    public static Vector<N2> offsetRprimeprime(SplineSE2 spline, double l, double s) {
        Pose2d pose = spline.pose(s);
        Rotation2d radial = pose.getRotation().plus(Rotation2d.k180deg);
        Rotation2d tangential = radial.plus(Rotation2d.kCCW_Pi_2);

        // centripetal a = r omega^2
        // dtheta/ds
        double headingRate = spline.dheading(s);
        // d^2x/ds^2 = x/theta * dtheta/ds * dtheta/ds
        double ca = l * headingRate * headingRate;
        // tangential
        double headingAccel = spline.ddheading(s);
        double ta = l * headingAccel;

        // note inversion of radial, it's pointing towards the center
        return VecBuilder.fill(
                -ca * radial.getCos() + ta * tangential.getCos(),
                -ca * radial.getSin() + ta * tangential.getSin());
    }

}
