package org.team100.battery_tester;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.battery_tester.Battery.Op;

public class EternalBatteryTest {
    @Test
    void test0() {
        // voltage at rest
        EternalBattery b = new EternalBattery();
        double v = b.V(0);
        assertEquals(12.6, v, 1e-3);
    }

    @Test
    void test1() {
        // moderate load, 100A, 10.6v = 1060 W.
        EternalBattery b = new EternalBattery();
        double v = b.V(100);
        assertEquals(10.6, v, 1e-3);
    }

    @Test
    void test2() {
        // high load, 300A, 6.6v => 1980W
        EternalBattery b = new EternalBattery();
        double v = b.V(300);
        assertEquals(6.6, v, 1e-3);
    }

    @Test
    void test3() {
        // operating point for zero power
        EternalBattery b = new EternalBattery();
        Op op = b.operatingPoint(0);
        assertEquals(0, op.i(), 1e-3);
        assertEquals(12.6, op.v(), 1e-3);
        assertEquals(0, op.i() * op.v(), 1e-3);
    }

    @Test
    void test4() {
        // operating point for 500 W
        EternalBattery b = new EternalBattery();
        Op op = b.operatingPoint(500);
        assertEquals(42.557, op.i(), 1e-3);
        assertEquals(11.749, op.v(), 1e-3);
        assertEquals(500.0, op.i() * op.v(), 0.1);
        assertEquals(11.749, b.V(42.557), 1e-3);
    }

    @Test
    void test5() {
        // operating point for 1000 W
        EternalBattery b = new EternalBattery();
        Op op = b.operatingPoint(1000);
        assertEquals(93.133, op.i(), 1e-3);
        assertEquals(10.737, op.v(), 1e-3);
        assertEquals(1000.0, op.i() * op.v(), 0.1);
        assertEquals(10.737, b.V(93.133), 1e-3);
    }
}
