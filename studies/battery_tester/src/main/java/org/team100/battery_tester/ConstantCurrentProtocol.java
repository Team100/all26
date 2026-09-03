package org.team100.battery_tester;

import org.team100.battery_tester.BatteryTester.Op;
import org.team100.lib.coherence.Takt;
import org.wpilib.command2.Command;

/** Run a full test and print the results. */
public class ConstantCurrentProtocol extends Command {

    /** summarize every 100 points, i.e. at 2 Hz. */
    private static final int DECIMATION = 100;
    private final BatteryTester m_subsystem;
    private final double m_i;
    private final double m_v;
    private Summarizer m_summarizer;
    private double m_start;

    /**
     * @param subsystem tester
     * @param i         target current, amperes
     * @param v         cutoff voltage, volts
     */
    public ConstantCurrentProtocol(
            BatteryTester subsystem, double i, double v) {
        m_subsystem = subsystem;
        m_i = i;
        m_v = v;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // nothing to initialize.
        m_start = Takt.get();
        m_summarizer = new Summarizer(DECIMATION);
    }

    @Override
    public void execute() {
        // System.out.println("current running");
        double t = Takt.get();
        double dt = t - m_start;
        double targetI = 0;
        if (dt < 3) {
            targetI = m_i * dt / 3;
        } else {
            targetI = m_i;
        }
        // System.out.printf("target I %f\n", targetI);
        m_subsystem.setCurrent(targetI);
        Op op = m_subsystem.operatingPoint();
        m_summarizer.add(t, op.inputI(), op.inputV(), op.p());
    }

    @Override
    public boolean isFinished() {
        Op op = m_subsystem.operatingPoint();
        // System.out.printf("isfinished op %s\n", op);
        return op.inputV() < m_v;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.off();
        m_summarizer.dump();
        summary();
    }

    private void summary() {
        double q = m_summarizer.charge();
        double t = m_summarizer.duration();
        // Peukert correction
        double k = 1.1641; // measured constant
        double i0 = 0.9; // 20h rate
        double ratio = Math.pow(i0 / m_i, k - 1);
        double qCorrected = q / ratio;
        System.out.printf("***********************************\n");
        System.out.printf("TEST COMPLETE\n");
        System.out.printf("TARGET CURRENT (AMPS)    %10.3f\n", m_i);
        System.out.printf("CUTOFF VOLTAGE (VOLTS)   %10.3f\n", m_v);
        System.out.printf("CHARGE (COULOMBS)        %10.3f\n", q);
        System.out.printf("CHARGE (AMP-HOURS)       %10.3f\n", q / 3600);
        System.out.printf("RATED CHARGE (COULOMBS)  %10.3f\n", qCorrected);
        System.out.printf("RATED CHARGE (AMP-HOURS) %10.3f\n", qCorrected / 3600);
        System.out.printf("DURATION (SECONDS)       %10.3f\n", t);
        System.out.printf("***********************************\n");
    }

}
