package org.team100.lib.gtsam.kinodynamics;



public class DriveUtil {
    public static Kinematics.SwerveModuleDeltas module_position_delta(
            Kinematics.SwerveModulePositions start,
            Kinematics.SwerveModulePositions end) {
        return new Kinematics.SwerveModuleDeltas(
                delta(start.front_left(), end.front_left()),
                delta(start.front_right(), end.front_right()),
                delta(start.rear_left(), end.rear_left()),
                delta(start.rear_right(), end.rear_right()));
    }

    public static Kinematics.SwerveModulePositions module_position_from_delta(
            Kinematics.SwerveModulePositions start, Kinematics.SwerveModuleDeltas delta) {
        return new Kinematics.SwerveModulePositions(
                plus(start.front_left(), delta.front_left()),
                plus(start.front_right(), delta.front_right()),
                plus(start.rear_left(), delta.rear_left()),
                plus(start.rear_right(), delta.rear_right()));
    }

    public static Kinematics.SwerveModuleDelta delta(
            Kinematics.SwerveModulePosition100 start,
            Kinematics.SwerveModulePosition100 end) {
        double delta_m = end.distance() - start.distance();
        return new Kinematics.SwerveModuleDelta(delta_m, end.angle());
    }

    static Kinematics.SwerveModulePosition100 plus(
            Kinematics.SwerveModulePosition100 start,
            Kinematics.SwerveModuleDelta delta) {
        double new_distance_m = start.distance() + delta.distance();
        return new Kinematics.SwerveModulePosition100(new_distance_m, delta.angle());
    }

}
