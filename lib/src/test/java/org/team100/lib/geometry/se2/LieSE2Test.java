package org.team100.lib.geometry.se2;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N3;

public class LieSE2Test {
    @Test
    void testInverse() {
        Matrix<N3, N3> m = LieSE2.hat(new Twist2d(1, 0, 1));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, -1, 1, //
                1, 0, 0, //
                0, 0, 0), m);
        Twist2d t = LieSE2.vee(m);
        TestUtil.verify(new Twist2d(1, 0, 1), t);
    }

    @Test
    void testBracket1() {
        // zero => zero
        Twist2d a = new Twist2d(0, 0, 0);
        Twist2d b = new Twist2d(1, 0, 1);
        Twist2d c = LieSE2.bracket(a, b);
        TestUtil.verify(new Twist2d(0, 0, 0), c);
    }

    @Test
    void testBracket2() {
        Twist2d a = new Twist2d(1, 0, 1);
        Twist2d b = new Twist2d(0, 1, 1);
        Twist2d c = LieSE2.bracket(a, b);
        TestUtil.verify(new Twist2d(-1, -1, 0), c);
    }
}
