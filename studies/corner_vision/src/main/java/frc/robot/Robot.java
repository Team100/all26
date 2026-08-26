package frc.robot;

import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.localization.AprilTagCornerRobotLocalizer;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.VisionUpdater;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.network.Sync;
import org.team100.lib.state.StateSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.visualization.RobotPoseVisualization;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
    private static final LoggerFactory logger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;

    private final Runnable m_robotViz;
    private final Sync sync;
    private final AprilTagCornerRobotLocalizer m_localizer;
    @SuppressWarnings("unused")
    private final FreshSwerveEstimate estimate;

    private Pose2d pose = new Pose2d();

    public Robot() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        sync = new Sync(inst);
        //
        LoggerFactory driveLog = logger.name("Drive");
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation.getLayout();
        DoubleFunction<StateSE2> history = (x) -> new StateSE2();
        m_localizer = new AprilTagCornerRobotLocalizer(
                driveLog,
                fieldLogger,
                layout,
                history,
                new VisionUpdater() {
                    @Override
                    public void put(
                            double timestampS,
                            NoisyPose2d noisyMeasurement) {
                        pose = noisyMeasurement.pose();
                    }
                },
                () -> Optional.of(Alliance.BLUE));
        estimate = new FreshSwerveEstimate(
                m_localizer::update,
                () -> {
                }, history);
        m_robotViz = new RobotPoseVisualization(
                fieldLogger, () -> pose, "robot");
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        sync.run();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_robotViz.run();
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }
}
