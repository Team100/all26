package org.team100.lib.subsystems.tank.commands;

import java.util.List;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.subsystems.tank.TankDrive;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Entry;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.TrajectorySE2Point;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** Given a goal pose, create a trajectory at initialization and follow it. */
public class ToPoseWithTrajectory extends Command {
    private final Pose2d m_goal;
    private final TankDrive m_drive;
    private final TrajectoryVisualization m_viz;
    private final TrajectorySE2Planner m_planner;
    private final LTVUnicycleController m_controller;

    private double m_startTimeS;
    private TrajectorySE2 m_trajectory;

    public ToPoseWithTrajectory(
            Pose2d goal,
            TankDrive drive,
            TrajectoryVisualization viz) {
        m_goal = goal;
        m_drive = drive;
        m_viz = viz;
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        m_planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        m_controller = new LTVUnicycleController(0.020);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_startTimeS = Takt.get();
        // may return null
        m_trajectory = m_planner.restToRest(List.of(
                WaypointSE2.tank(m_drive.getPose()),
                WaypointSE2.tank(m_goal)));
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
        try {
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
            ChassisSpeeds speeds = m_controller.calculate(
                    currentPose, poseReference, velocityReference, omegaReference);
            // accel feedforward
            AccelerationSE2 fieldRelativeAccel = nextControl.acceleration();
            ChassisAcceleration accel = ChassisAcceleration.fromFieldRelative(
                    fieldRelativeAccel, nextControl.rotation());
            m_drive.setVelocity(speeds, accel);
        } catch (IllegalStateException e) {
        }
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
