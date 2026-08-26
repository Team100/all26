package org.team100.battery_tester;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.battery_tester.CircuitUtil.Op;

public class CircuitUtilTest {
    @Test
    void test0() {
        // off
        EternalBattery b = new EternalBattery();
        LightBulb l = new LightBulb();
        CircuitUtil u = new CircuitUtil(l, b);
        Op op = u.operatingPointForDutyCycle(0);
        assertEquals(0, op.inputI(), 1e-3);
        assertEquals(12.6, op.inputV(), 1e-3);
    }

    @Test
    void test1() {
        EternalBattery b = new EternalBattery();
        LightBulb l = new LightBulb();
        CircuitUtil u = new CircuitUtil(l, b);
        Op op = u.operatingPointForDutyCycle(0.5);
        assertEquals(60.672, op.inputI(), 1e-3);
        assertEquals(11.386, op.inputV(), 1e-3);
    }

    @Test
    void test2() {
        EternalBattery b = new EternalBattery();
        LightBulb l = new LightBulb();
        CircuitUtil u = new CircuitUtil(l, b);
        Op op = u.operatingPointForDutyCycle(0.75);
        assertEquals(109.466, op.inputI(), 1e-3);
        assertEquals(10.411, op.inputV(), 1e-3);
    }

    @Test
    void test3() {
        // Maximum power possible
        EternalBattery b = new EternalBattery();
        LightBulb l = new LightBulb();
        CircuitUtil u = new CircuitUtil(l, b);
        Op op = u.operatingPointForDutyCycle(1.0);
        assertEquals(162.261, op.inputI(), 1e-3);
        assertEquals(9.355, op.inputV(), 1e-3);
    }

    @Test
    void test4() {
        // Low SOC, to exercise the difficult part.
        StatefulBattery b = new StatefulBattery();
        LightBulb l = new LightBulb();
        CircuitUtil u = new CircuitUtil(l, b);
        b.setC(6000);
        // Lower state of charge than we'd ever use,
        // but not so low that the solver fails.
        assertEquals(0.092, b.SOC(), 1e-3);
        // Low open circuit voltage
        assertEquals(11.22, b.V0(), 1e-3);
        // High internal resistance
        assertEquals(0.043, b.R(), 1e-3);
        Op op = u.operatingPointForDutyCycle(1.0);
        assertEquals(123.193, op.inputI(), 1e-3);
        assertEquals(5.842, op.inputV(), 1e-3);
    }

}
