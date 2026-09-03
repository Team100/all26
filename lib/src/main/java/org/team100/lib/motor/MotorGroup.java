package org.team100.lib.motor;

import java.util.function.Consumer;
import java.util.function.ToDoubleFunction;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.sensor.position.incremental.IncrementalEncoder;
import org.team100.lib.sensor.position.incremental.sim.SimulatedEncoder;

/** Treat a group of motors as a single motor. */
public class MotorGroup implements Motor {
    private final LoggerFactory m_log;
    private final Motor[] m_motors;

    public MotorGroup(LoggerFactory parent, Motor... motors) {
        m_log = parent.type(this);
        m_motors = motors;
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        apply((m) -> m.setTorqueLimit(torqueNm));
    }

    @Override
    public void setDutyCycle(double output) {
        apply((m) -> m.setDutyCycle(output));
    }

    @Override
    public void setVoltage(double voltage) {
        apply((m) -> m.setVoltage(voltage));
    }

    @Override
    public void setCurrent(double current) {
        apply((m) -> m.setCurrent(current));
    }

    @Override
    public void setVelocity(double velocityRad_S, double torqueNm) {
        apply((m) -> m.setVelocity(velocityRad_S, torqueNm));
    }

    @Override
    public double getUnwrappedPositionRad() {
        return mean((m) -> m.getUnwrappedPositionRad());
    }

    @Override
    public double getVelocityRad_S() {
        return mean((m) -> m.getVelocityRad_S());
    }

    @Override
    public double getAccelerationRad_S2() {
        return mean((m) -> m.getAccelerationRad_S2());
    }

    @Override
    public double getStatorCurrent() {
        return mean((m) -> m.getStatorCurrent());
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        apply((m) -> m.setUnwrappedEncoderPositionRad(positionRad));
    }

    @Override
    public void setUnwrappedPosition(double positionRad, double velocityRad_S, double torqueNm) {
        apply((m) -> m.setUnwrappedPosition(positionRad, velocityRad_S, torqueNm));
    }

    @Override
    public double R() {
        return mean((m) -> m.R());
    }

    @Override
    public double kT() {
        return mean((m) -> m.kT());
    }

    @Override
    public double kE() {
        return mean((m) -> m.kE());
    }

    @Override
    public IncrementalEncoder encoder() {
        return new SimulatedEncoder(m_log, this);
    }

    @Override
    public void stop() {
        apply((m) -> m.stop());
    }

    @Override
    public void reset() {
        apply((m) -> m.reset());
    }

    @Override
    public void close() {
        apply((m) -> m.close());
    }

    @Override
    public void periodic() {
        apply((m) -> m.periodic());
    }

    @Override
    public void play(double freq) {
        apply((m) -> m.play(freq));
    }

    private void apply(Consumer<Motor> f) {
        for (Motor m : m_motors) {
            f.accept(m);
        }
    }

    private double mean(ToDoubleFunction<Motor> f) {
        double v = 0;
        for (Motor m : m_motors) {
            v += f.applyAsDouble(m);
        }
        v /= m_motors.length;
        return v;
    }

    private double sum(ToDoubleFunction<Motor> f) {
        double v = 0;
        for (Motor m : m_motors) {
            v += f.applyAsDouble(m);
        }
        return v;
    }

    @Override
    public double getSupplyCurrent() {
        return sum(Motor::getSupplyCurrent);
    }

}
