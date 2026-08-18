package org.team100.lib.subsystems.turret;

import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;

import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.dynamics.r.RDynamicsAnalytic;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.r2.StateR2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.AccelLimitedVelocityProfileR1;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.LinearVelocityServo;
import org.team100.lib.servo.OnboardAngularPositionServo;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.FiringParameters;
import org.team100.lib.targeting.Solution;
import org.team100.lib.targeting.Solver;
import org.team100.lib.targeting.TimeOfFlightRecursion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Attempts to maintain aim. Demonstrates "spotting".
 * 
 * It provides Field2d visualization of the turret using the name "turret".
 * 
 * Now supports variable velocity.
 * 
 * TODO: add real motors.
 */
public class Turret extends SubsystemBase {
    // azimuth constants
    private static final double GEAR_RATIO = 100;
    private static final double MIN_POSITION = -3;
    private static final double MAX_POSITION = 3;
    // drum constants
    private static final double DRUM_RATIO = 1.0;
    private static final double DRUM_DIAMETER = 0.1;

    private final BooleanLogger m_log_aiming;
    private final BooleanLogger m_log_solved;
    private final DoubleArrayLogger m_log_field_turret;
    private final DoubleArrayLogger m_log_field_target;
    private final Supplier<ModelSE2> m_state;
    private final Supplier<Optional<Translation2d>> m_target;
    private final AngularPositionServo m_pivot;
    private final AngularPositionServo m_elevation;
    private final LinearVelocityServo m_drum;
    private final Solver m_solver;
    /** Indicates we're trying to find a solution and move there */
    private boolean m_aiming;
    /** Indicates we have a solution */
    private boolean m_solved;

    /**
     * @param parent Log
     * @param field  Field2d log
     * @param state  Current robot pose, from the pose estimator.
     * @param target From the target designator; can change at any time.
     */
    public Turret(
            LoggerFactory parent,
            LoggerFactory field,
            DoubleFunction<Optional<FiringParameters>> rangeToParams,
            Supplier<ModelSE2> state,
            Supplier<Optional<Translation2d>> target) {
        LoggerFactory log = parent.type(this);
        m_log_aiming = log.booleanLogger(Level.TRACE, "aiming");
        m_log_solved = log.booleanLogger(Level.TRACE, "solved");
        m_log_field_turret = field.doubleArrayLogger(Level.COMP, "turret");
        m_log_field_target = field.doubleArrayLogger(Level.COMP, "target");
        m_state = state;
        m_target = target;
        m_pivot = pivot(log.name("azimuth"));
        m_elevation = elevation(log.name("elevation"));
        m_drum = drum(log.name("drum"));
        // Laser solver always works
        // m_solver = new LaserSolver(rangeToParams);
        // TODO: make TOFR work
        m_solver = new TimeOfFlightRecursion(rangeToParams, 0.01);
        m_aiming = false;
        m_solved = false;
    }

    private static AngularPositionServo pivot(LoggerFactory log) {
        RDynamicsAnalytic dyn = new RDynamicsAnalytic(0.005);
        ProfileR1 profile = new TrapezoidProfileR1(5, 10, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);
        // TODO: real motor
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
        IncrementalBareEncoder encoder = motor.encoder();
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                log, encoder, GEAR_RATIO);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, GEAR_RATIO, MIN_POSITION, MAX_POSITION);
        AngularPositionServo pivot = new OnboardAngularPositionServo(
                log, mech, dyn, ref, feedback);
        pivot.reset();
        return pivot;
    }

    private static AngularPositionServo elevation(LoggerFactory log) {
        RDynamicsAnalytic dyn = new RDynamicsAnalytic(0, 0, 0, 0);
        ProfileR1 profile = new TrapezoidProfileR1(5, 10, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);
        // TODO: real motor
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
        IncrementalBareEncoder encoder = motor.encoder();
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                log, encoder, GEAR_RATIO);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, GEAR_RATIO, 0, Math.PI / 2);
        AngularPositionServo pivot = new OnboardAngularPositionServo(
                log, mech, dyn, ref, feedback);
        pivot.reset();
        return pivot;
    }

    private static LinearVelocityServo drum(LoggerFactory log) {
        PDynamics dyn = PDynamics.drum(0.0001, DRUM_DIAMETER / 2);
        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(10);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);
        // TODO: real motor
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
        IncrementalBareEncoder encoder = motor.encoder();
        LinearMechanism mech = new LinearMechanism(
                log, motor, encoder, DRUM_RATIO, DRUM_DIAMETER,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        LinearVelocityServo drum = new OutboardLinearVelocityServo(
                log, mech, dyn, ref, 1.0);
        return drum;
    }

    public boolean aiming() {
        return m_aiming;
    }

    public boolean solved() {
        return m_solved;
    }

    /** If false, the pivot has an inaccessible goal. */
    public boolean validSetpoint() {
        return m_pivot.validSetpoint();
    }

    public boolean onTarget() {
        return m_aiming && m_solved && m_pivot.atGoal() && m_elevation.atGoal() && m_drum.atGoal();
    }

    /** Absolute turret rotation */
    public Rotation2d getAzimuth() {
        Rotation2d relative = new Rotation2d(m_pivot.getWrappedPositionRad());
        return m_state.get().rotation().plus(relative);
    }

    public Rotation2d getElevation() {
        return new Rotation2d(m_elevation.getWrappedPositionRad());
    }

    public double getSpeed() {
        return m_drum.getVelocity();
    }

    private void moveToAim() {
        m_aiming = true;
        Optional<Solution> soln = getSolution();
        if (soln.isEmpty()) {
            // no solution is possible, don't do anything
            // TODO: keep drum spinning for awhile
            m_pivot.stop();
            m_elevation.stop();
            m_drum.stop();
            m_solved = false;
            return;
        }
        m_solved = true;
        m_log_solved.log(() -> true);
        Solution solution = soln.get();
        Rotation2d absoluteBearing = solution.azimuth();
        Rotation2d relativeBearing = absoluteBearing.minus(m_state.get().rotation());
        m_pivot.setPositionProfiled(relativeBearing.getRadians());
        m_elevation.setPositionProfiled(solution.parameters().elevation());
        m_drum.setVelocityProfiled(solution.parameters().speed());
    }

    private Optional<Solution> getSolution() {
        Optional<Translation2d> targetOpt = m_target.get();
        if (targetOpt.isEmpty())
            return Optional.empty();
        Translation2d targetTranslation = targetOpt.get();
        m_log_field_target.log(() -> new double[] {
                targetTranslation.getX(), targetTranslation.getY(), 0 });
        StateR2 target = new StateR2(targetTranslation, VelocityR2.ZERO);
        return m_solver.solve(m_state.get(), target);
    }

    private void stopAiming() {
        m_aiming = false;
        m_solved = false;
        m_pivot.stop();
        m_elevation.stop();
        m_drum.stop();
    }

    ////////////////////////////////////////////////////////
    ///
    /// COMMANDS

    public Command aim() {
        return run(this::moveToAim);
    }

    public Command stop() {
        return run(this::stopAiming);
    }

    @Override
    public void periodic() {
        m_pivot.periodic();
        m_elevation.periodic();
        m_drum.periodic();
        m_log_field_turret.log(this::poseArray);
        m_log_aiming.log(() -> m_aiming);
        m_log_solved.log(() -> m_solved);
    }

    private double[] poseArray() {
        Pose2d pose = m_state.get().pose();
        return new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().plus(
                        new Rotation2d(m_pivot.getWrappedPositionRad()))
                        .getDegrees() };
    }

}
