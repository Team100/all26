package org.team100.lib.util;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A collection with time-based eviction.
 * 
 * Eviction is separate from addition, because many times we have nothing to
 * add.
 */
public class TrailingHistory<T> {
    private static final boolean DEBUG = false;

    public record ValueRecord<T>(double time, T value) {
    };

    private final List<ValueRecord<T>> m_entries;

    public TrailingHistory() {
        m_entries = new ArrayList<ValueRecord<T>>();
    }

    /** Add the new value. */
    public void add(double time, T value) {
        if (DEBUG)
            System.out.printf("add %f %s\n", time, value);
        m_entries.add(new ValueRecord<>(time, value));
    }

    public List<T> getAll() {
        if (DEBUG)
            System.out.println("get all");
        return m_entries.stream()
                .map((x) -> {
                    if (DEBUG) {
                        System.out.printf("%s: %s\n", x.time, x.value);
                    }
                    return x.value;
                })
                .collect(Collectors.toUnmodifiableList());
    }

    /** Mutating iterator for filtering. */
    public Iterator<ValueRecord<T>> iterator() {
        return m_entries.iterator();
    }

    public int size() {
        return m_entries.size();
    }

    /**
     * Evict entries whose timestamps are earlier than the deadline.
     * 
     * This could take the current time and apply the delta, but it seems simpler to
     * let the caller figure out what deadline to use.
     * 
     * @param deadline seconds
     */
    public void evict(double deadline) {
        if (DEBUG)
            System.out.printf("eviction %f\n", deadline);
        m_entries.removeIf(x -> {
            boolean b = x.time < deadline;
            if (DEBUG)
                System.out.printf("evict %s %b\n", x, b);
            return b;
        });
    }

}
