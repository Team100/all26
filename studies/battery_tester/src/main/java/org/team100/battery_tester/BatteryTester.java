package org.team100.battery_tester;

import java.util.List;

import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.StateR1;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.motor.VictorSP;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.hardware.power.PowerDistribution.ModuleType;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.system.RobotController;

/**
 * Uses PWM controllers to extract a target power to the battery.
 * 
 * For now, all the output devices are controlled together.
 * There are 5 controllers with 3 bulbs each, so 15 bulbs,
 * effectively all parallel.
 */
public class BatteryTester extends SubsystemBase implements AutoCloseable {
    private static final boolean DEBUG = false;
    private final PowerDistribution m_pdh;
    private final List<VictorSP> m_controllers;
    private final FeedbackR1 m_feedback;
    private final LightBulb m_lightbulb;
    private final EternalBattery m_battery;
    private final StatefulBattery m_simBattery;
    final CircuitUtil m_circuit;

    private final DoubleLogger m_log_power;
    private final DoubleLogger m_log_desired_power;
    private final DoubleLogger m_log_desired_current;
    private final DoubleLogger m_log_ff;
    private final DoubleLogger m_log_fb;
    private final DoubleLogger m_log_t;
    private final DoubleLogger m_log_dutycycle;
    private final DoubleLogger m_log_output_voltage;
    private final DoubleLogger m_log_output_current;
    private final DoubleLogger m_log_output_power;
    private final DoubleLogger m_log_battery_voltage;
    private final DoubleLogger m_log_sim_battery_voltage;
    private final DoubleLogger m_log_soc;

    // Previously requested power (to avoid time travel).
    private double m_p;
    // Previously requested current.
    private double m_i;
    // Previously commanded dutycycle (to avoid discretization error).
    private double m_dutycycle;
    // Feedback accumulates, so the controller task is easier.
    private double m_fb;

    public BatteryTester(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        m_pdh = new PowerDistribution(0, 1, ModuleType.REV);
        m_controllers = List.of(        
                new VictorSP(0),
                new VictorSP(1),
                new VictorSP(2),
                new VictorSP(3),
                new VictorSP(4));
        m_feedback = new PIDFeedback(log, 0.00025, 0, 0.000001, false, 0.1, 1);
        m_lightbulb = new LightBulb();
        m_battery = new EternalBattery();
        m_simBattery = new StatefulBattery();
        m_circuit = new CircuitUtil(m_lightbulb, m_simBattery);
        m_log_power = log.doubleLogger(Level.DEBUG, "power (W)");
        m_log_desired_power = log.doubleLogger(Level.DEBUG, "desired power (W)");
        m_log_desired_current = log.doubleLogger(Level.DEBUG, "desired current (A)");
        m_log_ff = log.doubleLogger(Level.DEBUG, "ff");
        m_log_fb = log.doubleLogger(Level.DEBUG, "fb");
        m_log_t = log.doubleLogger(Level.DEBUG, "temperature (K)");
        m_log_dutycycle = log.doubleLogger(Level.DEBUG, "dutycycle");
        m_log_output_voltage = log.doubleLogger(Level.DEBUG, "output voltage (V)");
        m_log_output_current = log.doubleLogger(Level.DEBUG, "output current (A)");
        m_log_output_power = log.doubleLogger(Level.DEBUG, "output power (W)");
        m_log_battery_voltage = log.doubleLogger(Level.DEBUG, "battery voltage (V)");
        m_log_sim_battery_voltage = log.doubleLogger(Level.DEBUG, "sim battery voltage (V)");
        m_log_soc = log.doubleLogger(Level.DEBUG, "soc");
    }

    public void close() {
        m_controllers.forEach(VictorSP::close);
    }

    /** Set bulb power (watts). */
    public void setPower(double p) {
        m_log_desired_power.log(() -> p);
        // It also works without feedforward.
        // double ff = 0;
        double ff = ffPower(p);
        m_log_ff.log(() -> ff);
        // Feedback compares looks at the past; feedforward handles the present.
        double measurement = operatingPoint().p();
        double setpoint = m_p;
        double fb = m_feedback.calculate(
                new StateR1(measurement),
                new StateR1(setpoint));
        if (DEBUG) {
            System.out.printf("CurrentSource: measurement %f setpoint %f fb %f\n",
                    measurement, setpoint, fb);
        }
        m_p = p;
        m_log_fb.log(() -> fb);
        m_fb += fb;
        m_dutycycle = Math.clamp(ff + m_fb, 0, 1);
        m_controllers.stream().forEach(x -> x.setThrottle(m_dutycycle));
    }

    /** Set battery current (amps). */
    public void setCurrent(double i) {
        m_log_desired_current.log(() -> i);
        double ff = ffCurrent(i);
        m_log_ff.log(() -> ff);
        double measurement = operatingPoint().inputI();
        double setpoint = m_i;
        double fb = m_feedback.calculate(
                new StateR1(measurement),
                new StateR1(setpoint));
        m_i = i;
        m_log_fb.log(() -> fb);
        m_fb += fb;
        m_dutycycle = Math.clamp(ff + m_fb, 0, 1);
        m_controllers.stream().forEach(x -> x.setThrottle(m_dutycycle));
    }

    public void off() {
        setPower(0);
        setCurrent(0);
        m_feedback.reset();
        m_dutycycle = 0;
        m_fb = 0;
        m_controllers.stream().forEach(VictorSP::stopMotor);
    }

    /** Operating point of the battery */
    public record Op(double inputV, double inputI) {
        double p() {
            return inputV * inputI;
        }
    }

    /** Battery operating point. */
    public Op operatingPoint() {
        if (RobotBase.isReal()) {
            return inputOp();
        }
        return simOp();
    }

    /** Works with a real PDH. */
    private Op inputOp() {
        // battery voltage
        double v = batteryVoltage();
        // battery current
        double i = m_pdh.getTotalCurrent();
        return new Op(v, i);
    }

    /** Uses simulated battery. */
    private Op simOp() {
        CircuitUtil.Op op = m_circuit.operatingPointForDutyCycle(m_dutycycle);
        // bulb voltage
        // double v = m_dutycycle * batteryVoltage();
        // battery voltage
        double v = op.inputV();
        // bulb current
        // double i = lightbulb.IforV(v);
        // battery current
        double i = op.inputI();
        return new Op(v, i);
    }

    public double temperature() {
        Op op = operatingPoint();
        return m_lightbulb.temperature(op.p());
    }

    /**
     * Feedforward duty cycle.
     * 
     * @param p desired output power, watts
     */
    double ffPower(double p) {
        LightBulb.Op lbop = m_lightbulb.operatingPoint(p);
        Battery.Op bop = m_battery.operatingPoint(p);
        return Math.clamp(lbop.v() / bop.v(), 0, 1);
    }

    /**
     * Feedforward duty cycle
     * 
     * @param inputI desired battery current, amperes
     */
    double ffCurrent(double inputI) {
        double inputV = m_battery.V(inputI);
        double inputP = inputI * inputV;
        return ffPower(inputP);
    }

    public double batteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            CircuitUtil.Op op = m_circuit.operatingPointForDutyCycle(m_dutycycle);
            m_log_sim_battery_voltage.log(() -> op.inputV());
            RoboRioSim.setVInVoltage(op.inputV());
            m_simBattery.discharge(op.inputI(), TimedRobot100.LOOP_PERIOD_S);
            m_log_soc.log(m_simBattery::SOC);
        }
        Op op = operatingPoint();
        m_log_power.log(() -> op.p());
        m_log_t.log(this::temperature);
        m_log_dutycycle.log(() -> {
            return m_dutycycle;
        });
        m_log_output_voltage.log(() -> simOp().inputV());
        m_log_output_current.log(() -> simOp().inputI());
        m_log_output_power.log(() -> simOp().p());
        m_log_battery_voltage.log(this::batteryVoltage);
    }

}
