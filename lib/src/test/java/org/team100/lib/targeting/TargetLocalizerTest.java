package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

class TargetLocalizerTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void testCameraRotsToFieldRelativeArray() {
        // the camera rotations are all created the way the camera would do it,
        // with initial (x-ahead) and final (normalized pixels) vectors
        {
            // robot at origin
            Pose2d robotPose = new Pose2d();
            // camera level
            Transform3d camera = new Transform3d(new Translation3d(0, 0, 1), new Rotation3d());
            // sight on bore
            Rotation3d[] rots = new Rotation3d[] {
                    new Rotation3d(0, 0, 0)
            };
            List<Translation2d> translations = TargetLocalizer.cameraRotsToFieldRelativeArray(
                    robotPose, camera, rots);
            // sight on horizon, skip it
            assertEquals(0, translations.size());
        }
        {
            // robot at origin
            Pose2d robotPose = new Pose2d();
            // camera level
            Transform3d camera = new Transform3d(new Translation3d(0, 0, 1), new Rotation3d());
            // sight down 45
            Rotation3d[] rots = new Rotation3d[] {
                    new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 0, -1))
            };
            List<Translation2d> translations = TargetLocalizer.cameraRotsToFieldRelativeArray(
                    robotPose, camera, rots);
            assertEquals(1, translations.size());
            Translation2d t = translations.get(0);
            // camera is 1m high, pointing 45 down so 1m away
            assertEquals(1, t.getX(), DELTA);
            assertEquals(0, t.getY(), DELTA);
        }
        {
            // robot at origin
            Pose2d robotPose = new Pose2d();
            // camera level
            Transform3d camera = new Transform3d(new Translation3d(0, 0, 1), new Rotation3d());
            // sight down 45, left 45
            Rotation3d[] rots = new Rotation3d[] {
                    new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 1, -1))
            };
            List<Translation2d> translations = TargetLocalizer.cameraRotsToFieldRelativeArray(
                    robotPose, camera, rots);
            assertEquals(1, translations.size());
            Translation2d t = translations.get(0);
            // camera is 1m high, pointing 45 down so 1m away
            assertEquals(1, t.getX(), DELTA);
            assertEquals(1, t.getY(), DELTA);
        }
    }

    @Test
    void testcameraRotationToRobotRelative() {
        {
            // camera level
            Transform3d camera = new Transform3d(new Translation3d(0, 0, 1), new Rotation3d());
            // sight down 45
            Rotation3d sight = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 0, -1));
            Optional<Translation2d> target = TargetLocalizer.sightToRobotRelative(camera, sight);
            assertTrue(target.isPresent());
            Translation2d t = target.get();
            assertEquals(1, t.getX(), DELTA);
            assertEquals(0, t.getY(), DELTA);
        }
        {
            // camera down 45
            Transform3d camera = new Transform3d(new Translation3d(0, 0, 1), new Rotation3d(0, Math.PI / 4, 0));
            // sight on-bore
            Rotation3d sight = new Rotation3d();
            Optional<Translation2d> target = TargetLocalizer.sightToRobotRelative(camera, sight);
            assertTrue(target.isPresent());
            Translation2d t = target.get();
            assertEquals(1, t.getX(), DELTA);
            assertEquals(0, t.getY(), DELTA);
        }
        {
            // pitch 45 down, straight ahead, roll doesn't matter for this case.
            Transform3d camera = new Transform3d(new Translation3d(0, 0, 1), new Rotation3d(1, Math.PI / 4, 0));
            // sight on-bore
            Rotation3d sight = new Rotation3d();
            Optional<Translation2d> target = TargetLocalizer.sightToRobotRelative(camera, sight);
            assertTrue(target.isPresent());
            Translation2d t = target.get();
            assertEquals(1, t.getX(), DELTA);
            assertEquals(0, t.getY(), DELTA);
        }
        {
            // camera roll 90, pitch 45 down
            Transform3d camera = new Transform3d(
                    new Translation3d(0, 0, 1),
                    new Rotation3d(Math.PI / 2, Math.PI / 4, 0));
            // sight right 45
            Rotation3d sight = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, -1, 0));
            Optional<Translation2d> target = TargetLocalizer.sightToRobotRelative(camera, sight);
            assertTrue(target.isPresent());
            Translation2d t = target.get();
            // camera is rolled 90 right pitched 45 down, target is 45 more to the right
            // so target is at zero
            assertEquals(0, t.getX(), DELTA);
            assertEquals(0, t.getY(), DELTA);
        }
    }

    @Test
    void testRotation3d2() {
        // camera computes pitch and yaw as extrinsic euler angles of atan of pixels
        // which is wrong.
        // instead, find the rotation that maps the identity to the normalized pixels.
        {
            Vector<N3> initial = VecBuilder.fill(1, 0, 0);
            // 45 to the left
            Vector<N3> last = VecBuilder.fill(1, 1, 0);
            Rotation3d r = new Rotation3d(initial, last);
            assertEquals(0, r.getX(), DELTA);
            assertEquals(0, r.getY(), DELTA);
            assertEquals(Math.PI / 4, r.getZ(), DELTA);
        }
        {
            Vector<N3> initial = VecBuilder.fill(1, 0, 0);
            // 45 down
            Vector<N3> last = VecBuilder.fill(1, 0, -1);
            Rotation3d r = new Rotation3d(initial, last);
            assertEquals(0, r.getX(), DELTA);
            assertEquals(Math.PI / 4, r.getY(), DELTA);
            assertEquals(0, r.getZ(), DELTA);
        }
        {
            Vector<N3> initial = VecBuilder.fill(1, 0, 0);
            // 45 left and 45 down
            Vector<N3> last = VecBuilder.fill(1, 1, -1);
            Rotation3d r = new Rotation3d(initial, last);
            // roll we don't care about???
            assertEquals(0.261, r.getX(), DELTA);
            // less pitch because the yaw pulls it in
            assertEquals(0.615, r.getY(), DELTA);
            assertEquals(Math.PI / 4, r.getZ(), DELTA);

            // what happens if we use this in the intersection?
            // we want (1, 1)
            // camera level
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, 0, 0));
            Translation2d t = TargetLocalizer.sightToRobotRelative(camera, r).get();
            assertEquals(1, t.getX(), DELTA);
            assertEquals(1, t.getY(), DELTA);
        }
        {
            // as above with less tilt
            Vector<N3> initial = VecBuilder.fill(1, 0, 0);
            Vector<N3> last = VecBuilder.fill(1, 1, -0.5);
            Rotation3d r = new Rotation3d(initial, last);
            // roll we don't care about???
            assertEquals(0.142, r.getX(), DELTA);
            // less pitch because the yaw pulls it in
            assertEquals(0.340, r.getY(), DELTA);
            assertEquals(Math.PI / 4, r.getZ(), DELTA);

            // half the tilt == twice the distance
            // camera level
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, 0, 0));
            Translation2d t = TargetLocalizer.sightToRobotRelative(camera, r).get();
            assertEquals(2, t.getX(), DELTA);
            // y is affected the same which is correct
            assertEquals(2, t.getY(), DELTA);
        }
    }

    @Test
    void testRotation3d() {
        // constructor uses extrinsic rotations in x, y, z order,
        // so this pitches down and then yaws.
        Rotation3d r = new Rotation3d(0, Math.PI / 2, -Math.PI / 2);
        // we get the same angles out
        assertEquals(0, r.getX(), DELTA);
        assertEquals(Math.PI / 2, r.getY(), DELTA);
        assertEquals(-Math.PI / 2, r.getZ(), DELTA);
        // this is one of the ways to get back to zero
        Rotation3d r2 = r.rotateBy(new Rotation3d(-Math.PI / 2, 0, Math.PI / 2));
        assertEquals(0, r2.getX(), DELTA);
        assertEquals(0, r2.getY(), DELTA);
        assertEquals(0, r2.getZ(), DELTA);
    }

    @Test
    void testSightInRobotCoords() {
        // given a robot-relative camera offset and a camera-relative rotation,
        // produce the corresponding robot-relative transform
        {
            // camera level
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, 0, 0));
            // sight on-bore
            Rotation3d sight = new Rotation3d();
            Transform3d robotRelative = TargetLocalizer.sightInRobotCoords(camera, sight);
            assertEquals(0, robotRelative.getX(), DELTA);
            assertEquals(0, robotRelative.getY(), DELTA);
            assertEquals(1, robotRelative.getZ(), DELTA);
            // robot relative rotation is the same as the camera-relative translation since
            // the camera is parallel
            assertEquals(0, robotRelative.getRotation().getX(), DELTA);
            assertEquals(0, robotRelative.getRotation().getY(), DELTA);
            assertEquals(0, robotRelative.getRotation().getZ(), DELTA);
        }
        {
            // camera down 45
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.PI / 4, 0));
            // sight on-bore
            Rotation3d sight = new Rotation3d();
            Transform3d robotRelative = TargetLocalizer.sightInRobotCoords(camera, sight);
            assertEquals(0, robotRelative.getX(), DELTA);
            assertEquals(0, robotRelative.getY(), DELTA);
            assertEquals(1, robotRelative.getZ(), DELTA);
            // robot relative rotation is down 45
            assertEquals(0, robotRelative.getRotation().getX(), DELTA);
            assertEquals(Math.PI / 4, robotRelative.getRotation().getY(), DELTA);
            assertEquals(0, robotRelative.getRotation().getZ(), DELTA);
        }
        {
            // camera down 90, facing the floor
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.PI / 2, 0));
            // sight is on bore
            Rotation3d sight = new Rotation3d();
            Transform3d robotRelative = TargetLocalizer.sightInRobotCoords(camera, sight);
            assertEquals(0, robotRelative.getX(), DELTA);
            assertEquals(0, robotRelative.getY(), DELTA);
            assertEquals(1, robotRelative.getZ(), DELTA);
            //
            assertEquals(0.0, robotRelative.getRotation().getX(), DELTA);
            // this just yields the same thing as the camera
            assertEquals(1.571, robotRelative.getRotation().getY(), DELTA);
            //
            assertEquals(0.0, robotRelative.getRotation().getZ(), DELTA);
        }
        {
            // camera is pitched down 90, facing the floor
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.PI / 2, 0));
            // sight left 45
            Rotation3d sight = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 1, 0));
            Transform3d robotRelative = TargetLocalizer.sightInRobotCoords(camera, sight);
            assertEquals(0, robotRelative.getX(), DELTA);
            assertEquals(0, robotRelative.getY(), DELTA);
            assertEquals(1, robotRelative.getZ(), DELTA);
            // pi/2 roll, this is irrelevant
            assertEquals(1.571, robotRelative.getRotation().getX(), DELTA);
            // pi/4 pitch
            assertEquals(0.785, robotRelative.getRotation().getY(), DELTA);
            // pi/2 yaw
            assertEquals(1.571, robotRelative.getRotation().getZ(), DELTA);
            // result is 45 left, which is correct.
        }
        {
            // camera down 45
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.PI / 4, 0));
            // sight left 45
            Rotation3d sight = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 1, 0));
            Transform3d robotRelative = TargetLocalizer.sightInRobotCoords(camera, sight);
            assertEquals(0, robotRelative.getX(), DELTA);
            assertEquals(0, robotRelative.getY(), DELTA);
            assertEquals(1, robotRelative.getZ(), DELTA);
            // irrelevant roll
            assertEquals(0.615, robotRelative.getRotation().getX(), DELTA);
            // down less than 45, since the yaw pulls it down
            assertEquals(0.523, robotRelative.getRotation().getY(), DELTA);
            // left more than 45, due to the camera tilt
            assertEquals(0.955, robotRelative.getRotation().getZ(), DELTA);
        }
        {
            // this case is similar to the case above, down and left
            // but the camera is not rotated so the actual angles are different
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, 0, 0));
            // sight is down and to the left
            Rotation3d sight = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 1, -1));
            Transform3d robotRelative = TargetLocalizer.sightInRobotCoords(camera, sight);
            // robot relative translation is the same as the camera translation
            assertEquals(0, robotRelative.getX(), DELTA);
            assertEquals(0, robotRelative.getY(), DELTA);
            assertEquals(1, robotRelative.getZ(), DELTA);
            // irrelevant roll
            assertEquals(0.262, robotRelative.getRotation().getX(), DELTA);
            // down less than 45 because the yaw pulls it down
            assertEquals(0.615, robotRelative.getRotation().getY(), DELTA);
            // left exactly 45 because the camera is level
            assertEquals(0.785, robotRelative.getRotation().getZ(), DELTA);
        }
        {
            // camera is rolled 90 and pitched down 45
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(Math.PI / 2, Math.PI / 4, 0));
            // in camera sight is down 45 which means left 45 in the rotated frame
            Rotation3d sight = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 0, -1));
            Transform3d robotRelative = TargetLocalizer.sightInRobotCoords(camera, sight);
            // robot relative translation is the same as the camera translation
            assertEquals(0, robotRelative.getX(), DELTA);
            assertEquals(0, robotRelative.getY(), DELTA);
            assertEquals(1, robotRelative.getZ(), DELTA);
            // irrelevant roll
            assertEquals(2.186, robotRelative.getRotation().getX(), DELTA);
            // just like the non-rolled case
            assertEquals(0.523, robotRelative.getRotation().getY(), DELTA);
            // just like the non-rolled case.
            assertEquals(0.955, robotRelative.getRotation().getZ(), DELTA);
        }
    }

    @Test
    void testFloorIntersection() {
        {
            // camera 45 down
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.PI / 4, 0));
            // sight 45 left
            Rotation3d sight = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 1, 0));
            Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
            // bore is pointing at x=1
            assertEquals(1, t.getX(), DELTA);
            // distance to the floor is sqrt(2) so y is equal to that
            assertEquals(1.414, t.getY(), DELTA);
        }
        {
            // camera level
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, 0, 0));
            // target is down and to the left
            Rotation3d sight = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 1, -1));
            Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
            assertEquals(1, t.getX(), DELTA);
            assertEquals(1, t.getY(), DELTA);
        }
        {
            // camera level
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, 0, 0));
            // target is down and to the left
            Rotation3d sight = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 1, -0.5));
            Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
            // shallow angle means further away.
            assertEquals(2, t.getX(), DELTA);
            assertEquals(2, t.getY(), DELTA);
        }
        {
            // camera 90 down, facing the floor
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.PI / 2, 0));
            // sight is to the left
            Rotation3d sight = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 1, 0));
            Translation2d t = TargetLocalizer.sightToRobotRelative(camera, sight).get();
            assertEquals(0, t.getX(), DELTA);
            // camera altitude is 1, sight is 45 off bore, so Y is 1
            assertEquals(1, t.getY(), DELTA);
        }
    }

    @Test
    void testAboveHorizon() {
        // what happens above the horizon?
        {
            Transform3d robotRelative = new Transform3d(0, 0, 1, new Rotation3d(0, -1, 0));
            Optional<Translation2d> t = TargetLocalizer.sightInRobotCoordsToTranslation2d(robotRelative);
            assertTrue(t.isEmpty());
        }
        {
            Transform3d camera = new Transform3d(0, 0, 1, new Rotation3d(0, -Math.PI / 4, 0));
            Rotation3d sight = new Rotation3d(0, 0, Math.PI / 4);
            Optional<Translation2d> t = TargetLocalizer.sightToRobotRelative(camera, sight);
            assertTrue(t.isEmpty());
        }
    }

    @Test
    void testSightInRobotCoordsToTranslation2d() {
        // straight down should be zero
        Translation2d t = TargetLocalizer.sightInRobotCoordsToTranslation2d(
                new Transform3d(0, 0, 1, new Rotation3d(0, Math.PI / 2, 0))).orElseThrow();
        assertEquals(0, t.getX(), DELTA);
        assertEquals(0, t.getY(), DELTA);

        // 45 ahead
        t = TargetLocalizer.sightInRobotCoordsToTranslation2d(
                new Transform3d(0, 0, 1, new Rotation3d(0, Math.PI / 4, 0))).orElseThrow();
        assertEquals(1, t.getX(), DELTA);
        assertEquals(0, t.getY(), DELTA);

        // 45 left
        t = TargetLocalizer.sightInRobotCoordsToTranslation2d(
                new Transform3d(0, 0, 1, new Rotation3d(0, Math.PI / 4, Math.PI / 2))).orElseThrow();
        assertEquals(0, t.getX(), DELTA);
        assertEquals(1, t.getY(), DELTA);

        // 45 ahead constructed the way the camera does
        Rotation3d r = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 0, -1));
        assertEquals(0, r.getX(), DELTA);
        assertEquals(Math.PI / 4, r.getY(), DELTA);
        assertEquals(0, r.getZ(), DELTA);
        t = TargetLocalizer.sightInRobotCoordsToTranslation2d(
                new Transform3d(0, 0, 1, r)).orElseThrow();
        assertEquals(1, t.getX(), DELTA);
        assertEquals(0, t.getY(), DELTA);

        // oblique constructed the way the camera does
        r = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 1, -1));
        // note weird roll
        assertEquals(0.262, r.getX(), DELTA);
        assertEquals(0.615, r.getY(), DELTA);
        assertEquals(0.785, r.getZ(), DELTA);
        t = TargetLocalizer.sightInRobotCoordsToTranslation2d(
                new Transform3d(0, 0, 1, r)).orElseThrow();
        // but the answer turns out right
        assertEquals(1, t.getX(), DELTA);
        assertEquals(1, t.getY(), DELTA);

        // oblique constructed the way the camera does
        r = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 1, -0.5));
        assertEquals(0.142, r.getX(), DELTA);
        assertEquals(0.340, r.getY(), DELTA);
        assertEquals(0.785, r.getZ(), DELTA);
        t = TargetLocalizer.sightInRobotCoordsToTranslation2d(
                new Transform3d(0, 0, 1, r)).orElseThrow();
        assertEquals(2, t.getX(), DELTA);
        assertEquals(2, t.getY(), DELTA);

        // oblique constructed the way the camera does
        r = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, 0.5, -0.5));
        assertEquals(0.101, r.getX(), DELTA);
        assertEquals(0.420, r.getY(), DELTA);
        assertEquals(0.464, r.getZ(), DELTA);
        t = TargetLocalizer.sightInRobotCoordsToTranslation2d(
                new Transform3d(0, 0, 1, r)).orElseThrow();
        assertEquals(2, t.getX(), DELTA);
        assertEquals(1, t.getY(), DELTA);

        for (double y = -1; y <= 1; y += 0.1) {
            for (double z = -1; z < -0.1; z += 0.1) {
                if (DEBUG)
                    System.out.printf("y %6.3f z %6.3f\n", y, z);
                r = new Rotation3d(VecBuilder.fill(1, 0, 0), VecBuilder.fill(1, y, z));
                t = TargetLocalizer.sightInRobotCoordsToTranslation2d(
                        new Transform3d(0, 0, 1, r)).orElseThrow();
                assertEquals(-1 / z, t.getX(), DELTA);
                assertEquals(-y / z, t.getY(), DELTA);
            }
        }

    }

    @Test
    void testRobotRelativeToFieldRelative() {
        {
            Pose2d robotPose = new Pose2d();
            Translation2d cameraRotationRobotRelative = new Translation2d(1, 0);
            Translation2d t = TargetLocalizer.robotRelativeToFieldRelative(
                    robotPose,
                    cameraRotationRobotRelative);
            // since the robot is at the origin this doesn't do anything.
            assertEquals(1, t.getX(), DELTA);
            assertEquals(0, t.getY(), DELTA);
        }
        {
            Pose2d robotPose = new Pose2d(1, 0, new Rotation2d());
            Translation2d cameraRotationRobotRelative = new Translation2d(1, 0);
            Translation2d t = TargetLocalizer.robotRelativeToFieldRelative(
                    robotPose,
                    cameraRotationRobotRelative);
            // since the robot is at the origin this doesn't do anything.
            assertEquals(2, t.getX(), DELTA);
            assertEquals(0, t.getY(), DELTA);
        }
        {
            Pose2d robotPose = new Pose2d(1, 0, new Rotation2d(Math.PI / 4));
            Translation2d cameraRotationRobotRelative = new Translation2d(1, 0);
            Translation2d t = TargetLocalizer.robotRelativeToFieldRelative(
                    robotPose,
                    cameraRotationRobotRelative);
            // since the robot is at the origin this doesn't do anything.
            assertEquals(1.707, t.getX(), DELTA);
            assertEquals(0.707, t.getY(), DELTA);
        }
    }

}
