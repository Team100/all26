package org.team100.lib.geometry.so2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.Nat;

public class LieSO2Test {

    @Test
    void testInverse() {
        Matrix<N2, N2> m = LieSO2.hat(1);
        TestUtil.verify(MatBuilder.fill(Nat.N2(), Nat.N2(), //
                0, -1, //
                1, 0), m);
        double t = LieSO2.vee(m);
        assertEquals(1, t);
    }

}
