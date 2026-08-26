package org.team100.battery_tester;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class SummarizerTest {
    @Test
    void test0() {
        // verify one item is summarized correctly
        Summarizer r = new Summarizer(1);
        assertEquals(0, r.m_summary.n);
        r.add(2, 3, 4, 5);
        assertEquals(0, r.m_summary.n);
        assertEquals(1, r.m_result.size());
        Summary s = r.m_result.poll();
        assertEquals(0, s.tMax, 1e-3);
        assertEquals(3, s.iSum / s.n, 1e-3);
        assertEquals(4, s.vSum / s.n, 1e-3);
        assertEquals(5, s.pSum / s.n, 1e-3);
        assertEquals(0, s.qSum, 1e-3);
    }

    @Test
    void test1() {
        // verify two items are summarized correctly
        Summarizer r = new Summarizer(2);
        assertEquals(0, r.m_summary.n);
        r.add(2, 3, 4, 5);
        assertEquals(1, r.m_summary.n);
        r.add(3, 4, 5, 6);
        assertEquals(0, r.m_summary.n);
        assertEquals(1, r.m_result.size());
        Summary s = r.m_result.poll();
        assertEquals(1, s.tMax, 1e-3);
        assertEquals(3.5, s.iSum / s.n, 1e-3);
        assertEquals(4.5, s.vSum / s.n, 1e-3);
        assertEquals(5.5, s.pSum / s.n, 1e-3);
        // trapezoid integration
        assertEquals(3.5, s.qSum, 1e-3);
    }

    @Test
    void test2() {
        // two items summarized more often
        Summarizer r = new Summarizer(1);
        assertEquals(0, r.m_summary.n);
        r.add(2, 3, 4, 5);
        assertEquals(0, r.m_summary.n);
        r.add(3, 4, 5, 6);
        assertEquals(0, r.m_summary.n);
        assertEquals(2, r.m_result.size());
        // summarizing every item means you get it back verbatim
        Summary s = r.m_result.poll();
        assertEquals(0, s.tMax, 1e-3);
        assertEquals(3, s.iSum / s.n, 1e-3);
        assertEquals(4, s.vSum / s.n, 1e-3);
        assertEquals(5, s.pSum / s.n, 1e-3);
        // no dt, so no q
        assertEquals(0, s.qSum, 1e-3);
        // summarizing every item means you get it back verbatim
        s = r.m_result.poll();
        assertEquals(1, s.tMax, 1e-3);
        assertEquals(4, s.iSum / s.n, 1e-3);
        assertEquals(5, s.vSum / s.n, 1e-3);
        assertEquals(6, s.pSum / s.n, 1e-3);
        // trapezoid integration
        assertEquals(3.5, s.qSum, 1e-3);
    }

    @Test
    void test3() {
        // verify q is the cumulative sum
        Summarizer r = new Summarizer(1);
        r.add(1, 1, 12, 12);
        assertEquals(1, r.m_result.size());
        Summary s = r.m_result.peekLast();
        assertEquals(0, s.tMax, 1e-3);
        assertEquals(1, s.iSum / s.n, 1e-3);
        assertEquals(12, s.vSum / s.n, 1e-3);
        assertEquals(12, s.pSum / s.n, 1e-3);
        assertEquals(0, s.qSum, 1e-3);
        r.add(2, 1, 12, 12);
        assertEquals(2, r.m_result.size());
        s = r.m_result.peekLast();
        assertEquals(1, s.tMax, 1e-3);
        assertEquals(1, s.iSum / s.n, 1e-3);
        assertEquals(12, s.vSum / s.n, 1e-3);
        assertEquals(12, s.pSum / s.n, 1e-3);
        assertEquals(1, s.qSum, 1e-3);
        r.add(3, 1, 12, 12);
        assertEquals(3, r.m_result.size());
        s = r.m_result.peekLast();
        assertEquals(2, s.tMax, 1e-3);
        assertEquals(1, s.iSum / s.n, 1e-3);
        assertEquals(12, s.vSum / s.n, 1e-3);
        assertEquals(12, s.pSum / s.n, 1e-3);
        assertEquals(2, s.qSum, 1e-3);
    }

    @Test
    void test4() {
        // summarize every 50 points
        Summarizer r = new Summarizer(50);
        double t0 = 123.456;
        // 300 sec
        for (double t = 0; t < 300; t += 0.02) {
            r.add(t0 + t, 100, 12, 1200);
        }
        // should summarize every 1 sec
        assertEquals(300, r.m_result.size());
        r.dump();
    }

    @Test
    void test5() {
        // summarize every single point
        Summarizer r = new Summarizer(1);
        double t0 = 123.456;
        for (double t = 0; t < 1; t += 0.02) {
            r.add(t0 + t, 100, 12, 1200);
        }
        assertEquals(50, r.m_result.size());
        r.dump();
    }

    @Test
    void test6() {
        // summarize every fifth point
        Summarizer r = new Summarizer(5);
        double t0 = 123.456;
        for (double t = 0; t < 1; t += 0.02) {
            r.add(t0 + t, 100, 12, 1200);
        }
        assertEquals(10, r.m_result.size());
        r.dump();
    }

    @Test
    void test7() {
        // dump with no data => just the header.
        Summarizer r = new Summarizer(5);
        r.dump();
    }

}
