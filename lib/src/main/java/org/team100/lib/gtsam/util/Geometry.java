package org.team100.lib.gtsam.util;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;

import gtsam.Pose2;
import gtsam.Vector3;

public class Geometry {
    public static Pose2 toPose2(Pose2d p) throws Throwable {
        return new Pose2(p.getX(), p.getY(), p.getRotation().getRadians());
    }

    public static Pose2d toPose2d(Pose2 p) throws Throwable {
        return new Pose2d(p.x(), p.y(), new Rotation2d(p.theta()));
    }

    public static Twist2d twistFromVector(Vector3 v) throws Throwable {
        return new Twist2d(v.at(0), v.at(1), v.at(2));
    }

}
