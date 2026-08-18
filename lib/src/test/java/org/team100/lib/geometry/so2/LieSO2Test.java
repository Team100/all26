package org.team100.lib.geometry.so2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N2;

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
