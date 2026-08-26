package org.team100.battery_tester;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Accumulate measurements, summarize them periodically, and print the result.
 */
public class Summarizer {
    /** Summarize every n measurements. */
    private final int m_n;
    /** Series of summaries. */
    final Deque<Summary> m_result;
    /** Currently active accumulator. */
    Summary m_summary;

    /** @param n Summarize every n measurements. */
    public Summarizer(int n) {
        m_n = n;
        m_result = new ArrayDeque<>();
        m_summary = new Summary(null);
    }

    /** Add a measurement. */
    public void add(double t, double i, double v, double p) {
        m_summary.add(t, i, v, p);
        if (m_summary.n >= m_n) {
            m_result.add(m_summary);
            m_summary = new Summary(m_summary);
        }
    }

    public void dump() {
        System.out.flush();
        header();
        rows();
        System.out.flush();
    }

    public double charge() {
        Summary last = m_result.peekLast();
        if (last == null)
            return 0;
        return last.qSum;
    }

    public double duration() {
        Summary last = m_result.peekLast();
        if (last == null)
            return 0;
        return last.tMax;
    }

    /** Print the output table header. */
    private void header() {
        System.out.println("t, i, v, p, q");
    }

    /** Dump the contents of the buffer to stdout as a CSV. */
    private void rows() {
        m_result.stream().forEach(System.out::println);
    }
}
