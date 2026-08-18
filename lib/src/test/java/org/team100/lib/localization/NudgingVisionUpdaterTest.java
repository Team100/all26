package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class NudgingVisionUpdaterTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    /**
     * If the measurement is the same as the sample, nothing happens.
     */
    @Test
    void testZeroNudge() {
        // state is twice as confident as camera
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.02, 0.02);
        NoisyPose2d sample = new NoisyPose2d(new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(new Pose2d(), visionNoise);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(log, null, null);
        NoisyPose2d nudged = vu.nudge(sample, measurement);
        assertEquals(0, nudged.pose().getX(), 1e-6);
        assertEquals(0, nudged.pose().getY(), 1e-6);
        assertEquals(0, nudged.pose().getRotation().getRadians(), 1e-6);
        assertEquals(0.008944, nudged.noise().cartesian(), 1e-6);
        assertEquals(0.008944, nudged.noise().rotation(), 1e-6);
    }

    @Test
    void testOneGentleNudge() {
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.001, 0.1);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.1, Double.MAX_VALUE);
        NoisyPose2d sample = new NoisyPose2d(
                new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(0.1, 0, new Rotation2d(1)), visionNoise);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(log, null, null);
        NoisyPose2d nudged = vu.nudge(sample, measurement);
        assertEquals(0.000010, nudged.pose().getX(), 1e-6);
        assertEquals(0, nudged.pose().getY(), 1e-6);
        // infinite-variance vision update is completely ignored.
        assertEquals(0, nudged.pose().getRotation().getRadians(), 1e-6);
        assertEquals(0.003000, nudged.noise().cartesian(), 1e-6);
        // vision has infinite variance; this is the state variance.
        assertEquals(0.1, nudged.noise().rotation(), 1e-6);

    }

    /**
     * Let's say the camera is correct, and it says the sample is 0.1 meters off.
     * How long does it take for the pose buffer to contain the right pose? Kind of
     * a long time.
     */
    @Test
    void testGentleNudge() {
        int frameRate = 50;
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.1);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.05, Double.MAX_VALUE);
        NoisyPose2d sample = new NoisyPose2d(
                new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(0.1, 0, new Rotation2d()), visionNoise);
        NoisyPose2d nudged = sample;
        NudgingVisionUpdater vu = new NudgingVisionUpdater(log, null, null);
        for (int i = 0; i < frameRate; ++i) {
            nudged = vu.nudge(nudged, measurement);
        }
        // after 1 sec
        Transform2d error = measurement.pose().minus(nudged.pose());
        assertEquals(0.019, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
        //
        for (int i = 0; i < frameRate; ++i) {
            nudged = vu.nudge(nudged, measurement);
        }
        // after 2 sec
        error = measurement.pose().minus(nudged.pose());
        assertEquals(0.009, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
    }

    @Test
    void testOneFirmNudge() {
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.1);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.05, 0.5);
        NoisyPose2d sample = new NoisyPose2d(
                new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(0.1, 0, new Rotation2d()), visionNoise);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(log, null, null);
        NoisyPose2d nudged = vu.nudge(sample, measurement);
        assertEquals(0.003846, nudged.pose().getX(), 1e-6);
        assertEquals(0, nudged.pose().getY(), 1e-6);
        assertEquals(0, nudged.pose().getRotation().getRadians(), 1e-6);
        assertEquals(0.010176, nudged.noise().cartesian(), 1e-6);
        assertEquals(0.098058, nudged.noise().rotation(), 1e-6);

    }

    @Test
    void testOneFirm3dNudge() {
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.1);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.05, 0.5);
        NoisyPose2d sample = new NoisyPose2d(
                new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(1, 2, new Rotation2d(3)), visionNoise);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(log, null, null);
        NoisyPose2d nudged = vu.nudge(sample, measurement);
        assertEquals(0.038461, nudged.pose().getX(), 1e-6);
        assertEquals(0.076923, nudged.pose().getY(), 1e-6);
        assertEquals(0.115385, nudged.pose().getRotation().getRadians(), 1e-6);
        assertEquals(0.061598, nudged.noise().cartesian(), 1e-6);
        assertEquals(0.127562, nudged.noise().rotation(), 1e-6);

    }

    /**
     * Same as above with firmer updates. We want error under 2cm within about 1s.
     */
    @Test
    void testFirmerNudge() {
        int frameRate = 50;
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.1);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.05, Double.MAX_VALUE);
        NoisyPose2d sample = new NoisyPose2d(new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(0.1, 0, new Rotation2d()), visionNoise);
        NoisyPose2d nudged = sample;
        NudgingVisionUpdater vu = new NudgingVisionUpdater(log, null, null);
        for (int i = 0; i < frameRate; ++i) {
            nudged = vu.nudge(nudged, measurement);
        }
        // after 1 sec
        Transform2d error = measurement.pose().minus(nudged.pose());
        assertEquals(0.019, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
        //
        for (int i = 0; i < frameRate; ++i) {
            nudged = vu.nudge(nudged, measurement);
        }
        // after 2 sec
        error = measurement.pose().minus(nudged.pose());
        assertEquals(0.009372, error.getX(), 1e-6);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
    }

    /** Nudge across the angle modulus */
    @Test
    void testAngleCrossing() {
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.02, 0.02);
        NoisyPose2d sample = new NoisyPose2d(
                new Pose2d(0, 0, new Rotation2d(3)), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(0, 0, new Rotation2d(-3)), visionNoise);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(log, null, null);
        NoisyPose2d nudged = vu.nudge(sample, measurement);
        assertEquals(0, nudged.pose().getX(), 1e-6);
        assertEquals(0, nudged.pose().getY(), 1e-6);
        // nudged value is *more positive* even though the measurement is negative.
        assertEquals(3.056637, nudged.pose().getRotation().getRadians(), 1e-6);
        assertEquals(0.008944, nudged.noise().cartesian(), 1e-6);
        assertEquals(0.018347, nudged.noise().rotation(), 1e-6);

    }

    @Test
    void testNewState1() {
        // history thinks we're at 0, but it's not sure.
        ModelSE2 sampleModel = new ModelSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(1, 1);
        SwerveModulePositions positions = SwerveModulePositions.kZero();
        Rotation2d yaw = new Rotation2d();
        VariableR1 bias = VariableR1.fromVariance(0, 0.001);
        SwerveState sample = new SwerveState(
                sampleModel, stateNoise, positions, yaw, bias);

        // camera thinks we're at 1, and it's pretty sure.
        Pose2d visionPose = new Pose2d(1, 0, new Rotation2d());
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        NoisyPose2d measurement = new NoisyPose2d(visionPose, visionNoise);

        // result is close to the vision estimate, with its variance.
        NudgingVisionUpdater vu = new NudgingVisionUpdater(log, null, null);
        SwerveState newState = vu.newState(sample, measurement);
        assertEquals(1, newState.state().pose().getX(), DELTA);
        assertEquals(0, newState.state().pose().getY(), DELTA);
        assertEquals(0, newState.state().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.01, newState.noise().cartesian(), DELTA);
        assertEquals(0.01, newState.noise().rotation(), DELTA);
    }

    @Test
    void testNewState2() {
        // history thinks we're at 0, and it's pretty sure.
        ModelSE2 sampleModel = new ModelSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        SwerveModulePositions positions = SwerveModulePositions.kZero();
        Rotation2d yaw = new Rotation2d();
        VariableR1 bias = VariableR1.fromVariance(0, 0.001);
        SwerveState sample = new SwerveState(
                sampleModel, stateNoise, positions, yaw, bias);

        // camera thinks we're at 1, and it's pretty sure.
        Pose2d visionPose = new Pose2d(1, 0, new Rotation2d());
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        NoisyPose2d measurement = new NoisyPose2d(visionPose, visionNoise);

        // cartesian result is in the middle, with more variance.
        // rotation result is more confident, since history and camera agree.
        NudgingVisionUpdater vu = new NudgingVisionUpdater(log, null, null);
        SwerveState newState = vu.newState(sample, measurement);
        assertEquals(0.5, newState.state().pose().getX(), DELTA);
        assertEquals(0, newState.state().pose().getY(), DELTA);
        assertEquals(0, newState.state().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.071, newState.noise().cartesian(), DELTA);
        assertEquals(0.007, newState.noise().rotation(), DELTA);
    }

}
