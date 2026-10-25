package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.camera.Camera;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

class AprilTagReaderTest implements Timeless {
    private static final double DELTA = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testEndToEnd() throws IOException, InterruptedException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");

        AprilTagTranslator translator = new AprilTagTranslator(
                logger, layout, DriverStation::getAlliance);
        AprilTagReader reader = new AprilTagReader(
                logger, translator);

        // client instance
        NetworkTableInstance inst = NetworkTableInstance.create();
        inst.setServer("localhost");
        inst.startClient4("tag_finder24");

        // wait for the NT thread
        Thread.sleep(100);
        assertTrue(inst.isConnected());

        StructArrayTopic<Blip> topic = inst.getStructArrayTopic(
                "vision/origin/blips", Blip.struct);
        StructArrayPublisher<Blip> pub = topic.publish();
        // blip id=1
        pub.set(new Blip[] {
                Blip.fromXForward(0, 1, new Transform3d(1, 0, 0, new Rotation3d())) });

        // wait for NT rate-limiting
        Thread.sleep(200);
        inst.flush();

        // localizer needs alliance
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        List<VisionMeasurement> measurements = reader.read();
        assertTrue(measurements.isEmpty());

        // blip id=1
        // which is at (16.697, 0.655, 1.486), (0, 0, -0.94) in our coordinates
        // with zero camera offset the tag z is the tag elevation
        pub.set(new Blip[] {
                Blip.fromXForward(0, 1, new Transform3d(1.01, 0, 1.486, new Rotation3d())) });

        // wait for NT rate-limiting
        Thread.sleep(200);
        inst.flush();

        measurements = reader.read();
        assertEquals(1, measurements.size());

        Pose2d pose = measurements.get(0).noisyMeasurement().pose();

        // 1m away at -0.94 rad means 0.59 in x
        assertEquals(16.107, pose.getX(), DELTA);
        // and 0.807 in y
        assertEquals(1.472, pose.getY(), DELTA);
        // this is just the tag rotation
        assertEquals(-0.94, pose.getRotation().getRadians(), DELTA);
    }

    @Test
    void testEstimateRobotPose() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");

        AprilTagTranslator m_translator = new AprilTagTranslator(logger, layout, () -> Optional.of(Alliance.Red));

        // in red layout blip 7 is on the other side of the field

        // one meter range (Z forward)
        Blip blip = new Blip(0, 7,
                new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()));

        // verify tag location
        Pose3d tagPose = layout.getTagPose(Alliance.Red, 7).get();
        assertEquals(3.658, tagPose.getX(), DELTA);
        assertEquals(4.026, tagPose.getY(), DELTA);
        assertEquals(0.308, tagPose.getZ(), DELTA);
        assertEquals(0, tagPose.getRotation().getX(), DELTA);
        assertEquals(0, tagPose.getRotation().getY(), DELTA);
        assertEquals(0, tagPose.getRotation().getZ(), DELTA);

        // final String key = "foo";
        final Blip[] blips = new Blip[] {
                blip
        };

        Camera camera = Camera.UNKNOWN;
        List<VisionMeasurement> measurement = new ArrayList<>();

        measurement.addAll(m_translator.convert(camera, blips));
        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);

        Pose2d result = m.noisyMeasurement().pose();
        assertEquals(2.657, result.getX(), DELTA); // target is one meter in front
        assertEquals(4.026, result.getY(), DELTA); // same y as target
        assertEquals(0, result.getRotation().getRadians(), DELTA); // facing along x
    }

    @Test
    void testEstimateRobotPose2() throws IOException {
        // robot is panned right 45, translation is ignored.
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");

        AprilTagTranslator m_translator = new AprilTagTranslator(logger, layout, () -> Optional.of(Alliance.Red));

        // camera sees the tag straight ahead in the center of the frame,
        // but rotated pi/4 to the left. this is ignored anyway.
        // 1000 usec is 0.001 sec
        Blip blip = new Blip(1000, 7, new Transform3d(
                new Translation3d(0, 0, Math.sqrt(2)),
                new Rotation3d(0, -Math.PI / 4, 0)));

        // verify tag 7 location
        Pose3d tagPose = layout.getTagPose(Alliance.Red, 7).get();
        assertEquals(3.658, tagPose.getX(), DELTA);
        assertEquals(4.026, tagPose.getY(), DELTA);
        assertEquals(0.308, tagPose.getZ(), DELTA);
        assertEquals(0, tagPose.getRotation().getX(), DELTA);
        assertEquals(0, tagPose.getRotation().getY(), DELTA);
        assertEquals(0, tagPose.getRotation().getZ(), DELTA);

        final Blip[] blips = new Blip[] { blip };

        Camera camera = Camera.UNKNOWN;
        List<VisionMeasurement> measurement = new ArrayList<>();

        measurement.addAll(m_translator.convert(camera, blips));

        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);

        Pose2d result = m.noisyMeasurement().pose();
        // robot is is one meter away from the target in x
        assertEquals(2.658, result.getX(), DELTA);
        // robot is one meter to the left (i.e. in y)
        assertEquals(5.026, result.getY(), DELTA);
        // facing diagonal, this is just what we provided.
        assertEquals(-Math.PI / 4, result.getRotation().getRadians(), DELTA);

        assertEquals(0.001, m.timestamp(), DELTA);
    }

    @Test
    void testRotationInterpolation() {
        // just to be sure of what it's doing
        Rotation2d a = Rotation2d.fromDegrees(10);
        Rotation2d b = Rotation2d.fromDegrees(340);
        Rotation2d c = a.interpolate(b, 0.5);
        assertEquals(-5, c.getDegrees(), DELTA);
    }

    @Test
    void testCase1() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");

        AprilTagTranslator m_translator = new AprilTagTranslator(logger, layout, () -> Optional.of(Alliance.Red));

        Blip tag4 = new Blip(0, 4, new Transform3d(
                new Translation3d(0, 0, 2.4),
                new Rotation3d()));
        Blip tag3 = new Blip(0, 3, new Transform3d(
                new Translation3d(0.1, 0.1, 2.8),
                new Rotation3d()));

        final Blip[] tags = new Blip[] { tag3, tag4 };

        Camera camera = Camera.GAME_PIECE;
        List<VisionMeasurement> measurement = new ArrayList<>();
        measurement.addAll(m_translator.convert(camera, tags));
        measurement.addAll(m_translator.convert(camera, tags));
        // every input is ignored
        assertEquals(4, measurement.size());
    }

    @Test
    void testCase2() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        AprilTagTranslator m_translator = new AprilTagTranslator(logger, layout, () -> Optional.of(Alliance.Red));

        // tag is 1m away on bore
        final Blip tag4 = new Blip(0, 4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));

        final Blip[] tags = new Blip[] { tag4 };

        // if the tag is on bore then the camera is pretty high and also tilted up
        // the tag is tilted 30 degrees so the height is 0.5 meters lower than the tag
        Camera camera = Camera.TEST7A;
        List<VisionMeasurement> measurement = new ArrayList<>();
        measurement.addAll(m_translator.convert(camera, tags));
        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);
        assertEquals(8.272 - Math.sqrt(3) / 2, m.noisyMeasurement().pose().getX(), DELTA);
        assertEquals(1.914, m.noisyMeasurement().pose().getY(), DELTA);
    }

    @Test
    void testCase2WithOffset() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        AprilTagTranslator m_translator = new AprilTagTranslator(logger, layout, () -> Optional.of(Alliance.Red));

        Blip tag4 = new Blip(0, 4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));

        final Blip[] tags = new Blip[] { tag4 };

        // nonzero camera offset
        Camera camera = Camera.TEST7;
        List<VisionMeasurement> measurement = new ArrayList<>();

        measurement.addAll(m_translator.convert(camera, tags));
        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);
        assertEquals(7.272 - Math.sqrt(3) / 2, m.noisyMeasurement().pose().getX(), DELTA);
        assertEquals(1.914, m.noisyMeasurement().pose().getY(), DELTA);
    }

    @Test
    void testCase2tilt() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        AprilTagTranslator m_translator = new AprilTagTranslator(logger, layout, () -> Optional.of(Alliance.Red));

        Blip tag4 = new Blip(0, 4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        final Blip[] tags = new Blip[] { tag4 };

        Camera camera = Camera.TEST8;
        List<VisionMeasurement> measurement = new ArrayList<>();

        measurement.addAll(m_translator.convert(camera, tags));
        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);
        assertEquals(7.047, m.noisyMeasurement().pose().getX(), DELTA);
        assertEquals(1.914, m.noisyMeasurement().pose().getY(), DELTA);
    }

    @Test
    void testCase3() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        AprilTagTranslator m_translator = new AprilTagTranslator(logger, layout, () -> Optional.of(Alliance.Red));

        Blip tag4 = new Blip(0, 4, new Transform3d(
                new Translation3d(-1, 0, 1),
                new Rotation3d()));

        final Blip[] tags = new Blip[] { tag4 };

        Camera camera = Camera.UNKNOWN;
        List<VisionMeasurement> measurement = new ArrayList<>();
        measurement.addAll(m_translator.convert(camera, tags));
        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);
        assertEquals(7.407, m.noisyMeasurement().pose().getX(), DELTA);
        assertEquals(0.914, m.noisyMeasurement().pose().getY(), DELTA);
    }

    @Test
    void testCase4() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        AprilTagTranslator m_translator = new AprilTagTranslator(logger, layout, () -> Optional.of(Alliance.Red));

        Blip tag4 = new Blip(0, 4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        final Blip[] tags = new Blip[] { tag4 };

        Camera camera = Camera.UNKNOWN;
        List<VisionMeasurement> measurement = new ArrayList<>();
        measurement.addAll(m_translator.convert(camera, tags));
        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);
        assertEquals(7.047, m.noisyMeasurement().pose().getX(), DELTA);
        assertEquals(1.914, m.noisyMeasurement().pose().getY(), DELTA);
    }

    @Test
    void testCase5() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        AprilTagTranslator m_translator = new AprilTagTranslator(logger, layout, () -> Optional.of(Alliance.Red));

        Blip tag4 = new Blip(0, 4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        final Blip[] tags = new Blip[] { tag4 };
        Camera camera = Camera.UNKNOWN;
        List<VisionMeasurement> measurement = new ArrayList<>();
        measurement.addAll(m_translator.convert(camera, tags));
        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);
        assertEquals(7.047, m.noisyMeasurement().pose().getX(), DELTA);
        assertEquals(1.914, m.noisyMeasurement().pose().getY(), DELTA);
    }

    @Test
    void testCase6() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        AprilTagTranslator m_translator = new AprilTagTranslator(logger, layout, () -> Optional.of(Alliance.Red));

        Blip tag4 = new Blip(0, 4, new Transform3d(
                new Translation3d(0, 0, 2),
                new Rotation3d()));

        final Blip[] tags = new Blip[] { tag4 };

        Camera camera = Camera.TEST8;
        List<VisionMeasurement> measurement = new ArrayList<>();
        measurement.addAll(m_translator.convert(camera, tags));
        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);
        assertEquals(6.54, m.noisyMeasurement().pose().getX(), DELTA);
        assertEquals(1.914, m.noisyMeasurement().pose().getY(), DELTA);
    }

    @Test
    void testCase7() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        AprilTagTranslator m_translator = new AprilTagTranslator(logger, layout, () -> Optional.of(Alliance.Red));

        // 30 degrees, long side is sqrt2, so hypotenuse is sqrt2/sqrt3/2
        Blip tag4 = new Blip(0, 4, new Transform3d(
                new Translation3d(0, 0, 1.633),
                new Rotation3d()));

        final Blip[] tags = new Blip[] { tag4 };

        Camera camera = Camera.TEST9;
        List<VisionMeasurement> measurement = new ArrayList<>();

        measurement.addAll(m_translator.convert(camera, tags));
        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);
        assertEquals(6.858, m.noisyMeasurement().pose().getX(), DELTA);
        assertEquals(1.914, m.noisyMeasurement().pose().getY(), DELTA);
    }
}