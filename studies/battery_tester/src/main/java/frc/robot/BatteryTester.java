package frc.robot;

import java.util.List;

import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.ModelR1;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.motor.VictorSP;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.hardware.power.PowerDistribution.ModuleType;
import org.wpilib.math.util.MathUtil;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.system.RobotController;

/**
 * Uses PWM controllers to extract a target power to the battery.
 * 
 * For now, all the output devices are controlled together.
 * There are 5 controllers with 3 bulbs each, so 15 bulbs,
 * effectively all parallel.
 */
public class BatteryTester extends SubsystemBase {
    private static final boolean DEBUG = false;
    private final PowerDistribution pdh;
    private final List<VictorSP> controllers;
    private final FeedbackR1 feedback;
    private final LightBulb lightbulb;
    private final EternalBattery battery;
    private final StatefulBattery simBattery;
    private final CircuitUtil circuit;

    private final DoubleLogger m_log_power;
    private final DoubleLogger m_log_desired_power;
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
    // Previously commanded dutycycle (to avoid discretization error).
    private double m_dutycycle;
    // Feedback accumulates, so the controller task is easier.
    private double m_fb;

    public BatteryTester(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        pdh = new PowerDistribution(0, 1, ModuleType.REV);
        controllers = List.of(
                new VictorSP(0),
                new VictorSP(1),
                new VictorSP(2),
                new VictorSP(3),
                new VictorSP(4));
        feedback = new PIDFeedback(log, 0.00025, 0, 0.000001, false, 0.1, 1);
        lightbulb = new LightBulb();
        battery = new EternalBattery();
        simBattery = new StatefulBattery();
        circuit = new CircuitUtil(lightbulb, simBattery);
        m_log_power = log.doubleLogger(Level.DEBUG, "power (W)");
        m_log_desired_power = log.doubleLogger(Level.DEBUG, "desired power (W)");
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

    /** Set bulb power (watts). */
    public void setPower(double p) {
        m_log_desired_power.log(() -> p);
        // It also works without feedforward.
        // double ff = 0;
        double ff = ff(p);
        m_log_ff.log(() -> ff);
        // Feedback compares looks at the past; feedforward handles the present.
        double measurement = operatingPoint().p();
        double setpoint = m_p;
        double fb = feedback.calculate(
                new ModelR1(measurement),
                new ModelR1(m_p));
        if (DEBUG) {
            System.out.printf("CurrentSource: measurement %f setpoint %f fb %f\n",
                    measurement, setpoint, fb);
        }
        m_p = p;
        m_log_fb.log(() -> fb);
        m_fb += fb;
        m_dutycycle = Math.clamp(ff + m_fb, 0, 1);
        controllers.stream().forEach(x -> x.setThrottle(m_dutycycle));
    }

    public void off() {
        setPower(0);
        feedback.reset();
        m_dutycycle = 0;
        m_fb = 0;
        controllers.stream().forEach(VictorSP::stopMotor);
    }

    /** Operating point. */
    public record Op(double v, double i) {
        double p() {
            return v * i;
        }
    }

    public Op operatingPoint() {
        if (RobotBase.isReal()) {
            return inputOp();
        }
        return outputOp();
    }

    private Op inputOp() {
        // Only works with a real PDH.
        return new Op(batteryVoltage(), pdh.getTotalCurrent());
    }

    private Op outputOp() {
        double v = m_dutycycle * batteryVoltage();
        double i = lightbulb.IforV(v);
        return new Op(v, i);
    }

    public double temperature() {
        Op op = operatingPoint();
        return lightbulb.temperature(op.p());
    }

    /**
     * Feedforward duty cycle.
     * 
     * @param p desired output power, watts
     */
    private double ff(double p) {
        LightBulb.Op op = lightbulb.operatingPoint(p);
        // Battery voltage, including the sag from the required current.
        double vBatt = battery.V(op.i());
        return Math.clamp(op.v() / vBatt, 0, 1);
    }

    private double batteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            CircuitUtil.Op op = circuit.operatingPoint(m_dutycycle);
            m_log_sim_battery_voltage.log(() -> op.inputV());
            RoboRioSim.setVInVoltage(op.inputV());
            simBattery.discharge(op.inputI(), TimedRobot100.LOOP_PERIOD_S);
            m_log_soc.log(simBattery::SOC);
        }
        m_log_power.log(() -> operatingPoint().p());
        m_log_t.log(this::temperature);
        m_log_dutycycle.log(() -> {
            return m_dutycycle;
        });
        m_log_output_voltage.log(() -> outputOp().v());
        m_log_output_current.log(() -> outputOp().i());
        m_log_output_power.log(() -> outputOp().p());
        m_log_battery_voltage.log(this::batteryVoltage);
    }

}
