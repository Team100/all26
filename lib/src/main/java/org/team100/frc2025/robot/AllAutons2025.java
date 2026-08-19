package org.team100.frc2025.robot;

import java.util.List;

import org.team100.frc2025.field.FieldConstants2025.ReefPoint;
import org.team100.lib.config.AnnotatedCommandImpl;
import org.team100.lib.config.AutonChooser;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.FullStateControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;

import org.wpilib.command2.Command;

/**
 * Populates the Auton chooser with all available autons.
 * 
 * It's a good idea to instantiate them all here, even if you're not using them
 * all, even if they're just in development, so they don't rot.
 */
public class AllAutons2025 {
    private final AutonChooser m_autonChooser;

    public AllAutons2025(Machinery2025 machinery) {
        m_autonChooser = new AutonChooser();
        LoggerFactory autoLog = Logging.instance().rootLogger.name("Auton");

        final ProfileSE2 profile = HolonomicProfileFactory.currentLimitedExponential(1, 2, 4,
                machinery.m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                machinery.m_swerveKinodynamics.getMaxAngleAccelRad_S2(),
                5);
        final FullStateControllerSE2 controller = ControllerFactorySE2
                .auto2025LooseTolerance(autoLog);
        List<TimingConstraint> constraints = new TimingConstraintFactory(machinery.m_swerveKinodynamics)
                .medium();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        final TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

        // WARNING! The glass widget will override the default, so check it!
        // Run the auto in pre-match testing!
        m_autonChooser.addAsDefault(new AnnotatedCommandImpl(
                "Lollipop",
                new LolipopAuto(autoLog, machinery, profile, controller, planner).get()));

        DriveAndScore driveAndScore = new DriveAndScore(autoLog, machinery, profile, controller);
        m_autonChooser.add(new AnnotatedCommandImpl(
                "Coral 1 left",
                driveAndScore.get(ScoringLevel.L4, ReefPoint.J)));
        m_autonChooser.add(new AnnotatedCommandImpl(
                "Coral 1 mid",
                driveAndScore.get(ScoringLevel.L4, ReefPoint.H)));
        m_autonChooser.add(new AnnotatedCommandImpl(
                "Coral 1 right",
                driveAndScore.get(ScoringLevel.L4, ReefPoint.F)));

        Auton2025 auton = new Auton2025(autoLog, machinery, profile, controller);
        m_autonChooser.add(new AnnotatedCommandImpl(
                "Left Preload Only",
                auton.leftPreloadOnly()));
        m_autonChooser.add(new AnnotatedCommandImpl(
                "Center Preload Only",
                auton.centerPreloadOnly()));
        m_autonChooser.add(new AnnotatedCommandImpl(
                "Right Preload Only",
                auton.rightPreloadOnly()));
        m_autonChooser.add(new AnnotatedCommandImpl(
                "Left Three Coral",
                auton.left()));
        m_autonChooser.add(new AnnotatedCommandImpl(
                "Right Three Coral",
                auton.right()));
    }

    public Command get() {
        return m_autonChooser.get().command();
    }

    public void close() {
        m_autonChooser.close();
    }

}
