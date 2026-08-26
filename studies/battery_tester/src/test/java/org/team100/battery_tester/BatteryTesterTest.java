package org.team100.battery_tester;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

public class BatteryTesterTest {
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test0() {
        LightBulb lb = new LightBulb();
        Battery b = new EternalBattery();
        try (BatteryTester bt = new BatteryTester(log)) {
            // circuit for the ideal battery
            CircuitUtil m_circuit = new CircuitUtil(lb, b);

            double ff = bt.ffPower(500);
            assertEquals(0.395, ff, 1e-3);

            // the op here should match the ff since it's the ideal battery.
            CircuitUtil.Op op = m_circuit.operatingPointForDutyCycle(ff);
            assertEquals(42.552, op.inputI(), 1e-3);
            assertEquals(11.749, op.inputV(), 1e-3);
            assertEquals(500.0, op.inputI() * op.inputV(), 0.1);
            assertEquals(107.630, op.outputI(), 1e-3);
            assertEquals(4.645, op.outputV(), 1e-3);
            assertEquals(500.0, op.outputI() * op.outputV(), 0.1);
            assertEquals(0.395, op.outputV() / op.inputV(), 1e-3);
        }
    }

    @Test
    void test1() {
        try (BatteryTester bt = new BatteryTester(log);) {
            double ff = bt.ffCurrent(42.552);
            assertEquals(0.395, ff, 1e-3);
        }
    }
}
