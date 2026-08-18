package org.team100.lib.subsystems.test;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

/**
 * Demo of offset control, without actually changing any control
 * classes.
 * 
 * The controlled state is the "toolpoint" of the robot.
 * 
 * The drivetrain is the delegate, and its velocity commands are
 * derived from the toolpoint velocities using a fixed offset.
 * 
 * This version of the offset drivetrain does not include boosting.
 */
public class OffsetDrivetrain implements VelocitySubsystemSE2 {
    private final VelocitySubsystemSE2 m_delegate;
    private final Translation2d m_offset;

    /**
     * @param delgate the real drivetrain
     * @param offset  from delegate to toolpoint
     */
    public OffsetDrivetrain(
            VelocitySubsystemSE2 delegate, Translation2d offset) {
        m_delegate = delegate;
        m_offset = offset;
    }

    @Override
    public ModelSE2 getState() {
        return new ModelSE2(toolpointPose(), toolpointVelocity());
    }

    /**
     * Set delegate velocity from toolpoint velocity and offset.
     * r is from toolpoint to delegate, so invert offset.
     * 
     * TODO: the accel component is wrong. fix it.
     * 
     * @param nextV toolpoint velocity for the next timestep
     */
    @Override
    public void set(VelocityControlSE2 nextV) {
        // the component of the rotation part that tries to move the
        // delegate in x and y
        // respecting 100% of this velocity will keep the toolpoint
        // where it wants to go (if the delegate responds perfectly)
        VelocityControlSE2 tangentialVelocity = new VelocityControlSE2(
                OffsetUtil.tangentialVelocity(
                        OffsetUtil.omega(nextV.velocity()),
                        r(m_offset.unaryMinus())));

        m_delegate.set(nextV.plus(tangentialVelocity));
    }

    @Override
    public void stop() {
        m_delegate.stop();
    }

    /**
     * Computes toolpoint pose from delegate pose and offset.
     */
    private Pose2d toolpointPose() {
        return m_delegate.getState().pose().transformBy(
                new Transform2d(m_offset, Rotation2d.kZero));
    }

    /**
     * Computes toolpoint velocity from delegate velocity, pose, and offset.
     */
    private VelocitySE2 toolpointVelocity() {
        VelocitySE2 delegateVelocity = m_delegate.getState().velocity();
        return delegateVelocity.plus(
                OffsetUtil.tangentialVelocity(
                        OffsetUtil.omega(delegateVelocity), r(m_offset)));
    }

    private Rotation2d delegateRotation() {
        return m_delegate.getState().rotation();
    }

    /**
     * Vector form of the offset, rotated by the delegate pose rotation.
     */
    private Vector<N3> r(Translation2d offset) {
        return GeometryUtil.toVec3(
                offset.rotateBy(delegateRotation()));
    }

}
