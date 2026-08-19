package org.team100.lib.subsystems.tank.commands;

import java.util.function.Supplier;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.subsystems.tank.TankDrive;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Entry;
import org.team100.lib.trajectory.se2.TrajectorySE2Point;
import org.team100.lib.visualization.TrajectoryVisualization;

import org.wpilib.math.controller.LTVUnicycleController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command2.Command;

/**
 * Follows a trajectory with the WPI LTVUnicycleController, which combines
 * (reference velocity passthrough) feedforward and feedback. The feedback has
 * two independent components:
 * 
 * * translation: robot-relative X (fore/aft) error
 * * rotation: a mix of robot-relative Y (sideways) and rotation errors
 * 
 * The rotation error has a greater effect at greater speeds.
 */
public class FixedTrajectory extends Command {
    private final Supplier<TrajectorySE2> m_trajectorySupplier;
    private final TankDrive m_drive;
    private final TrajectoryVisualization m_viz;
    private final LTVUnicycleController m_controller;
    private double m_startTimeS;
    private TrajectorySE2 m_trajectory;

    public FixedTrajectory(
            Supplier<TrajectorySE2> trajectorySupplier,
            TankDrive drive,
            TrajectoryVisualization viz) {
        m_trajectorySupplier = trajectorySupplier;
        m_drive = drive;
        m_viz = viz;
        m_controller = new LTVUnicycleController(0.020);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_startTimeS = Takt.get();
        m_trajectory = m_trajectorySupplier.get();
        if (m_trajectory == null)
            return;
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
        if (m_trajectory == null)
            return;
        // current for position error
        double t = progress();
        TrajectorySE2Entry current = m_trajectory.sample(t);
        // next for feedforward (and selecting K)
        TrajectorySE2Entry next = m_trajectory.sample(t + TimedRobot100.LOOP_PERIOD_S);
        Pose2d currentPose = m_drive.getPose();
        Pose2d poseReference = current.point().point().waypoint().pose();
        // feedforward velocity
        TrajectorySE2Point nextPoint = next.point();
        ControlSE2 nextControl = nextPoint.control();
        double velocityReference = nextControl.velocity().norm();
        // feedforward velocity
        double omegaReference = nextControl.velocity().theta();
        // Control is proportional to pose error, producing CCW omega to fix +y,
        // -vx to fix +x. Includes the feedforward velocities.
        // It might be more correct to include the change in controller output in the
        // acceleration term.
        ChassisVelocities speeds = m_controller.calculate(
                currentPose, poseReference, velocityReference, omegaReference);
        // accel feedforward
        AccelerationSE2 fieldRelativeAccel = nextControl.acceleration();
        ChassisAcceleration accel = ChassisAcceleration.fromFieldRelative(
                fieldRelativeAccel, nextControl.rotation());
        m_drive.setVelocity(speeds, accel);
    }

    @Override
    public void end(boolean interrupted) {
        m_viz.clear();
    }

    /** Done when the timer expires. Ignores actual position */
    public boolean isDone() {
        return m_trajectory != null && m_trajectory.isDone(progress());
    }

    /** Time since start */
    private double progress() {
        return Takt.get() - m_startTimeS;
    }
}
