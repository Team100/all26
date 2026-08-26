package org.team100.battery_tester;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class LightBulbTest {

    @Test
    void test0() {
        // Array resistance at 3200K
        LightBulb b = new LightBulb();
        double r = b.R(3200.0);
        assertEquals(0.064170, r, 1e-6);
    }

    @Test
    void test1() {
        // Array power at 3200K.
        LightBulb b = new LightBulb();
        double p = b.radiation(3200.0);
        assertEquals(150 * 15, p, 1e-2);
    }

    @Test
    void test2() {
        // Array temperature at 150W (*15 bulbs)
        LightBulb b = new LightBulb();
        double t = b.temperature(150 * 15);
        assertEquals(3200, t, 1);
    }

    @Test
    void test3() {
        // Voltage required for 150W(*15 bulbs)
        LightBulb b = new LightBulb();
        double v = b.operatingPoint(150 * 15).v();
        assertEquals(12.0, v, 0.02);
    }

    @Test
    void test4() {
        // Sweep the power to see the voltage required
        LightBulb b = new LightBulb();
        for (double p = 0; p < 150 * 15; p += 100) {
            double t = b.temperature(p);
            double v = b.operatingPoint(p).v();
            double i = b.operatingPoint(p).i();
            System.out.printf("%f, %f, %f, %f\n", p, t, v, i);
        }
    }

    @Test
    void test5() {
        LightBulb b = new LightBulb();
        double i = b.IforV(5);
        assertEquals(112, i, 1);
    }

}
