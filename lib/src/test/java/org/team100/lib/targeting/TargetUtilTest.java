package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class TargetUtilTest {

    private static final double DELTA = 0.001;

    @Test
    void testUnwrappedAbsoluteBearing() {
        // no translation => zero bearing
        // this depends on the bad behavior of Rotation2d, where norm of zero yields angle of zero
        assertEquals(0, TargetUtil.unwrappedAbsoluteBearing(new Pose2d(), new Translation2d()), DELTA);
        assertEquals(0, TargetUtil.unwrappedAbsoluteBearing(
                new Pose2d(0, 0, new Rotation2d(1)), new Translation2d()), DELTA);
    }

    @Test
    void testBearing() {
        assertEquals(0,
                TargetUtil.absoluteBearing(
                        new Translation2d(),
                        new Translation2d(1, 0)).getRadians(),
                DELTA);
        assertEquals(Math.PI / 2,
                TargetUtil.absoluteBearing(
                        new Translation2d(),
                        new Translation2d(0, 1)).getRadians(),
                DELTA);
        assertEquals(Math.PI / 4,
                TargetUtil.absoluteBearing(
                        new Translation2d(),
                        new Translation2d(1, 1)).getRadians(),
                DELTA);
        assertEquals(3 * Math.PI / 4,
                TargetUtil.absoluteBearing(
                        new Translation2d(),
                        new Translation2d(-1, 1)).getRadians(),
                DELTA);
        assertEquals(-Math.PI / 4,
                TargetUtil.absoluteBearing(
                        new Translation2d(),
                        new Translation2d(1, -1)).getRadians(),
                DELTA);
    }

    @Test
    void testTargetMotion() {
        // at the origin moving 1 m/s +x
        ModelSE2 state = new ModelSE2(new Pose2d(), new VelocitySE2(1, 0, 0));
        // target is 1m to the left
        Translation2d target = new Translation2d(0, 1);
        // so it appears to move clockwise
        assertEquals(1, TargetUtil.targetMotion(state, target), DELTA);
    }

    @Test
    void testTargetMotionFaster() {
        // at the origin moving 2 m/s +x
        ModelSE2 state = new ModelSE2(new Pose2d(), new VelocitySE2(2, 0, 0));
        // target is 1m to the left
        Translation2d target = new Translation2d(0, 1);
        // so it appears to move clockwise
        assertEquals(2, TargetUtil.targetMotion(state, target), DELTA);
    }

    @Test
    void testTargetMotionElsewhere() {
        // somewhere else, moving 1 m/s +x
        ModelSE2 state = new ModelSE2(new Pose2d(1, 1, new Rotation2d()), new VelocitySE2(1, 0, 0));
        // target is 1m to the left
        Translation2d target = new Translation2d(1, 2);
        // so it appears to move clockwise
        assertEquals(1, TargetUtil.targetMotion(state, target), DELTA);
    }

    @Test
    void testTargetMotionReverse() {
        // at the origin, moving 1m/s +x
        ModelSE2 state = new ModelSE2(new Pose2d(), new VelocitySE2(1, 0, 0));
        // target is 1m to the right
        Translation2d target = new Translation2d(0, -1);
        // so it appears to move counterclockwise
        assertEquals(-1, TargetUtil.targetMotion(state, target), DELTA);
    }

    @Test
    void testTargetMotionAhead() {
        // at the origin, moving 1m/s +x
        ModelSE2 state = new ModelSE2(new Pose2d(), new VelocitySE2(1, 0, 0));
        // target is dead ahead
        Translation2d target = new Translation2d(2, 0);
        // no apparent motion
        assertEquals(0, TargetUtil.targetMotion(state, target), DELTA);
    }

    @Test
    void testTargetMotionOblique() {
        // at the origin, moving 1m/s +x
        ModelSE2 state = new ModelSE2(new Pose2d(), new VelocitySE2(1, 0, 0));
        // target is at 45
        Translation2d target = new Translation2d(1, 1);
        // apparent motion is slower
        assertEquals(0.5, TargetUtil.targetMotion(state, target), DELTA);
    }

    @Test
    void testTargetMotionY() {
        // at the origin, moving 1m/s +y
        ModelSE2 state = new ModelSE2(new Pose2d(), new VelocitySE2(0, 1, 0));
        // target is dead ahead
        Translation2d target = new Translation2d(1, 0);
        // target moves the other way
        assertEquals(-1, TargetUtil.targetMotion(state, target), DELTA);
    }

    @Test
    void testTargetMotionYReversed() {
        // in front of the origin, facing back to it, moving 1m/s +y,
        ModelSE2 state = new ModelSE2(
                new Pose2d(1, 0, Rotation2d.kPi),
                new VelocitySE2(0, 1, 0));
        // target is dead ahead
        Translation2d target = new Translation2d(0, 0);
        // target appears to move counterclockwise
        assertEquals(1, TargetUtil.targetMotion(state, target), DELTA);
    }

    @Test
    void testTargetMotionZero() {
        // not moving, no motion
        ModelSE2 state = new ModelSE2(new Pose2d(), new VelocitySE2(0, 0, 0));
        // target is 1m to the left
        Translation2d target = new Translation2d(0, 1);
        // it should not move
        assertEquals(0, TargetUtil.targetMotion(state, target), DELTA);
    }
}
