package org.team100.lib.subsystems.se2.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.reference.se2.ProfileReferenceSE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.VelocityReferenceControllerSE2;
import org.team100.lib.targeting.TargetUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Drive to a 2d goal position, while aiming at it.
 * 
 * This is intended for autonomous driving with an intake, to make sure the
 * intake is oriented correctly before arrival, It should seem like a
 * "pushbroom" motion, so the intake is roughly perpendicular to the approach.
 * 
 * This is different from driving to the goal Pose2d, because there's no goal
 * rotation: the rotation goal is just based on the relative translation.
 * 
 * The goal is the bearing, but the deadband depends on range, effectively
 * acting like a constant width around the boresight.
 * 
 * This is a near-copy of DriveToTranslationWithRelativeBearing, but I want to
 * leave that alone since Vasili was working on it.
 */
public class Pushbroom extends MoveAndHold {
    /** ignore goals further than this from the previous fix */
    private static final double GOAL_UPDATE_RADIUS = 0.2;
    /**
     * Theta deadband uses lateral distance. This should be some fraction of the
     * lateral tolerance of the intake. For example, if your intake is 0.4 meters
     * wide, you might want this command to aim for the middle 0.2 meters or so, or
     * 0.1 meters either side of center.
     */
    private static final double LATERAL_TOLERANCE = 0.1;
    private final LoggerFactory m_log;
    private final DoubleArrayLogger m_log_field_goal;
    private final DoubleArrayLogger m_log_field_target;
    private final Supplier<Optional<Translation2d>> m_target;
    private final VelocitySubsystemSE2 m_drive;
    private final ControllerSE2 m_controller;
    private final ProfileSE2 m_profile;
    /** End effector relative to robot */
    private final Transform2d m_endEffector;

    private Translation2d target;
    private Pose2d m_goal;
    private ProfileReferenceSE2 m_reference;
    private VelocityReferenceControllerSE2 m_referenceController;

    public Pushbroom(
            LoggerFactory parent,
            LoggerFactory field,
            Supplier<Optional<Translation2d>> target,
            VelocitySubsystemSE2 drive,
            ControllerSE2 controller,
            ProfileSE2 profile,
            Transform2d endEffector) {
        m_log = parent.type(this);
        m_log_field_goal = field.doubleArrayLogger(Level.COMP, "goal");
        m_log_field_target = field.doubleArrayLogger(Level.COMP, "target");
        m_target = target;
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_endEffector = endEffector;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        target = m_target.get().orElse(null);
        updateGoal();
        if (m_goal == null)
            return;
        m_reference = new ProfileReferenceSE2(m_log, m_profile, "DriveToTranslationWithRelativeBearing");
        m_reference.setGoal(new ModelSE2(m_goal));
        m_referenceController = new VelocityReferenceControllerSE2(
                m_log, m_drive, m_controller, m_reference);
    }

    @Override
    public void execute() {
        // update the goal during execution because the camera is more accurate when the
        // target is closer.
        target = m_target.get().orElse(null);
        updateGoal();
        if (m_goal == null || m_referenceController == null)
            return;
        m_reference.setGoal(new ModelSE2(m_goal));
        m_referenceController.execute();
        m_log_field_goal.log(() -> new double[] {
                m_goal.getX(),
                m_goal.getY(),
                m_goal.getRotation().getDegrees() });
        m_log_field_target.log(() -> new double[] {
                target.getX(),
                target.getY(),
                0 });
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
        m_goal = null;
    }

    private void updateGoal() {

        if (target == null)
            return;
        Pose2d robot = m_drive.getState().pose();
        Pose2d candidate = candidate(
                m_reference == null ? robot : m_reference.current().pose(),
                target, robot, m_endEffector);
        m_goal = filterGoal(candidate);
    }

    static Pose2d candidate(
            Pose2d prev, Translation2d targetTranslation, Pose2d robotPose, Transform2d endEffectorOffset) {
        Pose2d endEffectorPose = prev == null ? robotPose.plus(endEffectorOffset) : prev.plus(endEffectorOffset);
        // Pose2d endEffectorPose = robotPose.plus(endEffectorOffset);
        Translation2d endEffectorTranslation = endEffectorPose.getTranslation();
        Rotation2d endEffectorRotation = endEffectorPose.getRotation();
        double endEffectorToTargetRange = targetTranslation.getDistance(endEffectorTranslation);
        Rotation2d endEffectorBearingToTarget = TargetUtil.absoluteBearing(endEffectorTranslation, targetTranslation);
        Rotation2d endEffectorToGo = endEffectorBearingToTarget.minus(endEffectorRotation);
        double lateral = endEffectorToTargetRange * endEffectorToGo.getSin();
        if (endEffectorToGo.getCos() > 0 && Math.abs(lateral) < LATERAL_TOLERANCE) {
            // stop steering, use the previous
            // cos restriction prevents going backwards.
            endEffectorBearingToTarget = endEffectorPose.getRotation();
        }
        Pose2d targetEndEffectorPose = new Pose2d(targetTranslation, endEffectorBearingToTarget);
        Pose2d targetRobotPose = targetEndEffectorPose.plus(endEffectorOffset.inverse());
        return targetRobotPose;
    }

    /**
     * Sometimes we lose sight of the actual closest target, and choose a further
     * away one. In that case, we should ignore the update.
     */
    private Pose2d filterGoal(Pose2d candidate) {
        if (m_goal == null) {
            return candidate;
        }
        if (m_goal.getTranslation().getDistance(candidate.getTranslation()) < GOAL_UPDATE_RADIUS) {
            return candidate;
        }
        return m_goal;
    }
}
