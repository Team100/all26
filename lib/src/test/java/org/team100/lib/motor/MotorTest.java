package org.team100.lib.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class MotorTest {

    @Test
    void test0() {
        Motor m = new AbstractMotor() {
            @Override
            public double kE() {
                return 1;
            }
        };
        assertEquals(1, m.backEMFVoltage(1), 0.001);

    }

}
