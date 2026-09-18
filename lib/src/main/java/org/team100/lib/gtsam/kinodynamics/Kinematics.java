package org.team100.lib.gtsam.kinodynamics;

import java.util.List;

import org.ejml.simple.SimpleMatrix;

import gtsam.Vector3;

/**
 * This is so i don't need to import all the Team100/WPI stuff about odometry.
 */
public class Kinematics {
    public static record RotR2(double c, double s) {
        public static RotR2 fromDegrees(double x) {
            double xr = Math.toRadians(x);
            return new RotR2(Math.cos(xr), Math.sin(xr));
        }

        public static RotR2 fromXY(double x, double y) {
            double mag = Math.hypot(x, y);
            if (mag > 1e-6)
                return new RotR2(x / mag, y / mag);
            return new RotR2(1, 0);
        }
    }

    public static record SwerveModulePosition100(
            double distance,
            RotR2 angle) {
    }

    public static record SwerveModulePositions(
            SwerveModulePosition100 front_left,
            SwerveModulePosition100 front_right,
            SwerveModulePosition100 rear_left,
            SwerveModulePosition100 rear_right) {
    }

    public static record PointR2(double x, double y) {
    }

    public static class SwerveDriveKinematics100 {
        int num_modules;
        List<PointR2> module_locations;
        SimpleMatrix inverse_kinematics;
        SimpleMatrix forward_kinematics;

        public SwerveDriveKinematics100(List<PointR2> module_translations) {
            num_modules = module_translations.size();
            module_locations = module_translations;
            inverse_kinematics = inverse_matrix(module_translations);
            forward_kinematics = inverse_kinematics.pseudoInverse();
        }

        /** FORWARD: module deltas -> twist. */
        public Twist2d to_twist_2d(SwerveModuleDeltas deltas) {
            // [d cos; d sin; ...] (2n x 1)
            SimpleMatrix delta_vector = deltas_2_vector(deltas);
            // [dx ;dy; dtheta]
            SimpleMatrix twist_vector = forward_kinematics.mult(delta_vector);
            return vector_2_twist(twist_vector);
        }

        /**
         * """
         * INVERSE: twist -> module position deltas
         * 
         * This assumes the wheel paths are geodesics; steering does not change.
         * """
         * 
         * @return
         */
        public SwerveModuleDeltas to_swerve_module_delta(Twist2d twist) {

            // [dx; dy; dtheta] (3 x 1)
            SimpleMatrix twist_vector = twist_2_vector(twist);
            // [d cos; d sin; ...] (2n x 1)
            SimpleMatrix delta_vector = inverse_kinematics.mult(twist_vector);
            return deltas_from_vector(delta_vector);
        }

        /** Find the module deltas and apply them to the given initial positions. */
        public SwerveModulePositions to_swerve_module_positions(
                SwerveModulePositions initial, Twist2d twist) {

            SwerveModuleDeltas deltas = to_swerve_module_delta(twist);
            return DriveUtil.module_position_from_delta(initial, deltas);
        }

        /** deltas -> [d cos; d sin; ... ] (2n x 1) */
        private SimpleMatrix deltas_2_vector(SwerveModuleDeltas module_deltas) {
            SimpleMatrix module_delta_matrix = new SimpleMatrix(num_modules * 2, 1);
            List<SwerveModuleDelta> module_deltas_all = module_deltas.all();
            for (int i = 0; i < num_modules; ++i) {
                SwerveModuleDelta module = module_deltas_all.get(i);
                if (Math.abs(module.distance()) < 1e-6) {
                    module_delta_matrix.set(i * 2, 0, 0);
                    module_delta_matrix.set(i * 2 + 1, 0, 0);
                } else {
                    module_delta_matrix.set(i * 2, 0,
                            module.distance * module.angle.c());
                    module_delta_matrix.set(i * 2 + 1, 0,
                            module.distance * module.angle.s());
                }
            }
            return module_delta_matrix;
        }

        private static SimpleMatrix twist_2_vector(Twist2d twist) {
            SimpleMatrix m = new SimpleMatrix(3, 1);
            m.set(0, 0, twist.x);
            m.set(1, 0, twist.y);
            m.set(2, 0, twist.theta);
            return m;
        }

        /**
         * The resulting distance is always positive.
         * 
         * @param moduleDeltaVector [d cos; d sin; ...] (2n x 1),
         *                          equivalently [dx0; dy0; dx1; ...]
         */
        private static SwerveModuleDeltas deltas_from_vector(
                SimpleMatrix module_delta_vector) {

            return new SwerveModuleDeltas(
                    SwerveModuleDelta.of(module_delta_vector.get(0, 0), module_delta_vector.get(1, 0)),
                    SwerveModuleDelta.of(module_delta_vector.get(2, 0), module_delta_vector.get(3, 0)),
                    SwerveModuleDelta.of(module_delta_vector.get(4, 0), module_delta_vector.get(5, 0)),
                    SwerveModuleDelta.of(module_delta_vector.get(6, 0), module_delta_vector.get(7, 0)));
        }

        private static Twist2d vector_2_twist(SimpleMatrix v) {
            return new Twist2d(v.get(0, 0), v.get(1, 0), v.get(2, 0));
        }

        private static SimpleMatrix inverse_matrix(List<PointR2> module_locations) {
            int num_modules = module_locations.size();
            SimpleMatrix invk = new SimpleMatrix(num_modules * 2, 3);
            for (int i = 0; i < num_modules; ++i) {
                invk.set(i * 2 + 0, 0, 1);
                invk.set(i * 2 + 0, 1, 0);
                invk.set(i * 2 + 0, 2, -module_locations.get(i).y());
                invk.set(i * 2 + 1, 0, 0);
                invk.set(i * 2 + 1, 1, 1);
                invk.set(i * 2 + 1, 2, module_locations.get(i).x());
            }
            return invk;
        }
    }

    public static record Twist2d(double x, double y, double theta) {
        public Twist2d() {
            this(0, 0, 0);
        }

        public static Twist2d fromVector(Vector3 v) throws Throwable {
            return new Twist2d(v.at(0), v.at(1), v.at(2));
        }
    }

    public static record SwerveModuleDelta(
            double distance,
            RotR2 angle) {
        public static SwerveModuleDelta of(double x, double y) {
            return new SwerveModuleDelta(
                    Math.hypot(x, y),
                    RotR2.fromXY(x, y));
        }
    }

    public static record SwerveModuleDeltas(
            SwerveModuleDelta front_left,
            SwerveModuleDelta front_right,
            SwerveModuleDelta rear_left,
            SwerveModuleDelta rear_right) {
        public List<SwerveModuleDelta> all() {
            return List.of(front_left, front_right, rear_left, rear_right);
        }
    }
}
