package frc.robot;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.LinearPositionServo;
import org.team100.lib.servo.OutboardLinearPositionServo;
import org.team100.lib.util.CanId;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

public class ClimberExtension extends SubsystemBase {
    private final LinearPositionServo m_servo;

    private final double m_maxExtensionM = 0.25;
    private final double m_minextension = 0.01;

    public ClimberExtension(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory log = parent.type(this);
        ProfileR1 profile = new TrapezoidProfileR1(0.1, 2, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        double wheelDiameterM = 0.001275;
        int gearRatio = 1;
        PDynamics dyn = new PDynamics(0);

        switch (Identity.instance) {
            case COMP_BOT -> {
                CurrentLimit limit = new CurrentLimit(40, 40);
                NeoVortexCANSparkMotor m_motor = new NeoVortexCANSparkMotor(
                        log,
                        currentLog,
                        new CanId(2),
                        NeutralMode100.BRAKE,
                        MotorPhase.FORWARD,
                        limit,
                        new Friction(0, 0, 0, 0),
                        new PIDConstants(1, 0, 0, 0, 0, 0),
                        0,
                        0);
                IncrementalBareEncoder encoder = m_motor.encoder();
                LinearMechanism climberMech = new LinearMechanism(
                        log, m_motor, encoder, gearRatio, wheelDiameterM,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OutboardLinearPositionServo(
                        log, climberMech, dyn, ref, 0.01, 0.01);
            }

            default -> {
                SimulatedBareMotor m_motor = new SimulatedBareMotor(log, 600);
                IncrementalBareEncoder encoder = m_motor.encoder();
                LinearMechanism climberMech = new LinearMechanism(
                        log, m_motor, encoder, gearRatio, wheelDiameterM,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OutboardLinearPositionServo(
                        log, climberMech, dyn, ref, 0.01, 0.01);
            }
        }
    }

    public Command setPosition() {
        return run(this::setOutPosition);
    }

    public Command setHomePosition() {
        return run(this::setInPosition);
    }

    public void setOutPosition() {
        m_servo.setPositionProfiled(m_maxExtensionM);

    }

    public void setInPosition() {
        m_servo.setPositionProfiled(m_minextension);
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }
}
