package org.team100.battery_tester;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class StatefulBatteryTest {

    @Test
    void test0() {
        // voltage
        StatefulBattery b = new StatefulBattery();
        assertEquals(12, b.ocv.get(0.5), 1e-3);
    }

    @Test
    void test1() {
        // resistance
        StatefulBattery b = new StatefulBattery();
        assertEquals(0.021, b.r.get(0.5), 1e-3);
    }

    @Test
    void test2() {
        // Peukert derating.
        StatefulBattery b = new StatefulBattery();
        assertEquals(18.0, b.peukert(0) * b.c0 / 3600, 0.001);
        // 20h rating
        assertEquals(18.0, b.peukert(0.9) * b.c0 / 3600, 0.001);
        // 10h rating, note this doesn't match very well
        assertEquals(17.0, b.peukert(1.7) * b.c0 / 3600, 0.79);
        // 5h rating, note this doesn't match very well at all.
        assertEquals(15.7, b.peukert(3.14) * b.c0 / 3600, 1.05);
        // 1h rating, I weighed this point the highest.
        assertEquals(11.8, b.peukert(11.8) * b.c0 / 3600, 0.001);
        // check that it's in [0,1]
        assertEquals(0.655, b.peukert(11.8), 0.001);
    }

    @Test
    void test3() {
        StatefulBattery b = new StatefulBattery();
        double dt = 0.02;
        // discharge at 20h rate for 1h.
        for (double t = 0; t < 3600; t += dt) {
            b.discharge(0.9, dt);
        }
        // 5% of the duration at the rated rate, so 5% discharged.
        assertEquals(0.95, b.SOC(), 1e-3);
    }

    @Test
    void test4() {
        StatefulBattery b = new StatefulBattery();
        double dt = 0.02;
        // discharge at 1h rate for 0.5h.
        for (double t = 0; t < 1800; t += dt) {
            b.discharge(11.8, dt);
        }
        // 50% of the duration at the rated rate, so 50% discharged.
        assertEquals(0.5, b.SOC(), 1e-3);
    }

    @Test
    void test5() {
        StatefulBattery b = new StatefulBattery();
        double dt = 0.02;
        // discharge at a typical FRC rate for a few minutes
        // TODO: realistic FRC load, which is more variable
        for (double t = 0; t < 160; t += dt) {
            b.discharge(140, dt);
            // System.out.printf("t %f soc %f\n", t, b.soc());
        }
        // 80% discharged after 2:40. (!)
        assertEquals(0.2, b.SOC(), 1e-2);
    }
}
