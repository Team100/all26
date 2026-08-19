package org.team100.lib.subsystems.se2.commands;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.FullStateControllerSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.se2.MockSubsystemSE2;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.YawRateConstraint;
import org.team100.lib.visualization.TrajectoryVisualization;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.Command;

/**
 * Demonstrate DriveWithTrajectoryFunction.
 * 
 * https://docs.google.com/spreadsheets/d/1tt7Fq-gkR7aoY6kH2WFVxj4y__SMXiKE0scPG3eHSAk/edit?gid=0#gid=0
 */
public class DriveWithTrajectoryFunctionTest implements Timeless {

    private static final boolean DEBUG = false;

    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    MockSubsystemSE2 subsystem = new MockSubsystemSE2(new ModelSE2());
    FullStateControllerSE2 controller = ControllerFactorySE2.test(log);
    TrajectoryVisualization viz = new TrajectoryVisualization(log);
    List<TimingConstraint> constraints = List.of(
            new ConstantConstraint(2, 2),
            new YawRateConstraint(1, 1));
    PathSE2Factory pathFactory = new PathSE2Factory();
    TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
    TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

    /**
     * This is the key to using DriveWithTrajectoryFunction: a function that takes a
     * starting pose and returns a trajectory.
     */
    TrajectorySE2 makeTrajectory(Pose2d startingPose) {
        return planner.restToRest(
                List.of(
                        WaypointSE2.irrotational(startingPose, 0, 1.2),
                        WaypointSE2.irrotational(
                                new Pose2d(1, 2, new Rotation2d(Math.PI / 2)), Math.PI / 2, 1.2)));
    }

    @Test
    void testDemo() {
        Command drive = new DriveWithTrajectoryFunction(
                log, subsystem, controller, viz, this::makeTrajectory);
        drive.initialize();
        stepTime();
        if (DEBUG)
            System.out.println("x, y, theta");
        for (int i = 0; i < 200; ++i) {
            drive.execute();
            subsystem.m_state = new ModelSE2(
                    subsystem.m_state.pose(),
                    subsystem.m_setpoint.velocity());
            subsystem.m_state = subsystem.m_state.evolve(0.02);
            Pose2d p = subsystem.m_state.pose();
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f\n", p.getX(), p.getY(), p.getRotation().getRadians());
            stepTime();
        }
    }
}
