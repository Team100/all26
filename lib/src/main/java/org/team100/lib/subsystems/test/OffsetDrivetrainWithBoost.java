package org.team100.lib.subsystems.test;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;

import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.numbers.N3;

/**
 * Demo of offset control, without actually changing any control
 * classes.
 * 
 * The controlled state is the "toolpoint" of the robot.
 * 
 * The drivetrain is the delegate, and its velocity commands are
 * derived from the toolpoint velocities using a fixed offset.
 * 
 * This version of the offset drivetrain includes "boosting", which means that
 * some of the toolpoint desired velocity is perpendicular to the offset, and so
 * by adding a rotation to the delegate, we can move the toolpoint in its
 * desired direction a bit faster, in exchange for some theta error. This
 * essentially edits the output of the controller, so we can leave the
 * controller alone.
 */
public class OffsetDrivetrainWithBoost implements VelocitySubsystemSE2 {
    /**
     * How much of the perpendicular speed to mix in. This interacts with the
     * controller "P" values, so should be tuned together with them.
     */
    private static final double OMEGA_MIXER = 2.0;
    private final VelocitySubsystemSE2 m_delegate;
    private final Translation2d m_offset;

    /**
     * @param delgate the real drivetrain
     * @param offset  from delegate to toolpoint
     */
    public OffsetDrivetrainWithBoost(
            VelocitySubsystemSE2 delegate, Translation2d offset) {
        m_delegate = delegate;
        m_offset = offset;
    }

    @Override
    public StateSE2 getState() {
        return new StateSE2(toolpointPose(), toolpointVelocity());
    }

    /**
     * Set delegate velocity from toolpoint velocity and offset.
     * r is from toolpoint to delegate, so invert offset.
     * 
     * @param nextV toolpoint velocity for the next timestep
     */
    @Override
    public void set(VelocityControlSE2 nextV) {
        VelocitySE2 nextVelocity = nextV.velocity();
        // The component of the cartesian part that tries to spin
        // the delegate.
        // Adding some of this will make the toolpoint move more rapidly
        // towards the cartesian goal, while injecting theta error.
        Vector<N3> reverseR = offset(m_offset);
        Vector<N3> cartesian = OffsetUtil.velocity(nextVelocity);
        VelocitySE2 perpendicularOmega = OffsetUtil.omega(
                reverseR, cartesian);

        // The component of the rotation part that tries to move the
        // delegate in x and y.
        // Respecting 100% of this velocity will keep the toolpoint
        // where it wants to go (if the delegate responds perfectly)
        Vector<N3> omega = OffsetUtil.omega(nextVelocity);
        Vector<N3> r = offset(m_offset.unaryMinus());
        VelocitySE2 tangentialVelocity = OffsetUtil.tangentialVelocity(
                omega, r);
        AccelerationSE2 centripetalAccel = OffsetUtil.centripetalAcceleration(
                omega, r);
        VelocitySE2 totalVelocity = tangentialVelocity.plus(perpendicularOmega.times(OMEGA_MIXER));
        VelocityControlSE2 control = new VelocityControlSE2(
                totalVelocity, centripetalAccel);
        m_delegate.set(nextV.plus(control));
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
                        OffsetUtil.omega(delegateVelocity), offset(m_offset)));
    }

    private Rotation2d delegateRotation() {
        return m_delegate.getState().rotation();
    }

    /**
     * Vector form of the offset, rotated by the delegate pose rotation.
     */
    private Vector<N3> offset(Translation2d offset) {
        return GeometryUtil.toVec3(
                offset.rotateBy(delegateRotation()));
    }

}
