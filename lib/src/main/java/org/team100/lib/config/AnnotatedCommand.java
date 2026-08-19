package org.team100.lib.config;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.trajectory.se2.TrajectorySE2;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.Command;

/**
 * A command with annotations that are checked against ground truth while
 * disabled.
 */
public interface AnnotatedCommand {

    /**
     * Must be unique.
     */
    String name();

    /**
     * Command to run
     */
    Command command();

    /**
     * Red or blue, null if it works for both.
     */
    default Alliance alliance() {
        return null;
    }

    /**
     * Starting pose, null if it doesn't matter.
     */
    default Pose2d start() {
        return null;
    }

    /**
     * For visualization.
     * 
     * Starting point => trajectory
     * Chain these together to see the whole path.
     */
    List<Function<Pose2d, TrajectorySE2>> trajectoryFns();
}
