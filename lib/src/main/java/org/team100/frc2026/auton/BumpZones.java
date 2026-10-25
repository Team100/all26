package org.team100.frc2026.auton;

import java.util.List;

import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.VelocityLimitRegionConstraint;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BumpZones {
    public static final Rectangle2d BLUE_BUMP_LEFT = new Rectangle2d(
            new Translation2d(5.2, 6.43),
            new Translation2d(4.1, 4.6));

    public static final Rectangle2d BLUE_BUMP_RIGHT = new Rectangle2d(
            new Translation2d(5.2, 3.2),
            new Translation2d(4.1, 1.64));

    public static final Rectangle2d RED_BUMP_RIGHT = new Rectangle2d(
            new Translation2d(12.5, 6.43),
            new Translation2d(11.32, 4.6));

    public static final Rectangle2d RED_BUMP_LEFT = new Rectangle2d(
            new Translation2d(12.5, 3.2),
            new Translation2d(11.32, 1.64));

    public static List<TimingConstraint> constraint(double v) {
        VelocityLimitRegionConstraint c1 = new VelocityLimitRegionConstraint(
                BumpZones.BLUE_BUMP_LEFT, v);
        VelocityLimitRegionConstraint c2 = new VelocityLimitRegionConstraint(
                BumpZones.BLUE_BUMP_RIGHT, v);
        VelocityLimitRegionConstraint c3 = new VelocityLimitRegionConstraint(
                BumpZones.RED_BUMP_LEFT, v);
        VelocityLimitRegionConstraint c4 = new VelocityLimitRegionConstraint(
                BumpZones.RED_BUMP_RIGHT, v);
        return List.of(c1, c2, c3, c4);
    }

}
