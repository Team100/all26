package org.team100.lib.coherence;

/**
 *
 * Represents an action that should be coherent, even though its only effects
 * are side-effects, i.e. there's no direct output to cache.
 * 
 * For example, when the vision reader does its thing, there's no simple output
 * to cache, only mutations to the pose history, which can be picked up by
 * querying the history itself. Use this class to represent the action that
 * should be refreshed.
 */
public class SideEffect implements Runnable {
    private final Runnable m_delegate;
    private boolean m_valid;

    /** Do not use this, use Cache.ofSideEffect(). */
    SideEffect(Runnable delegate) {
        m_delegate = delegate;
        m_valid = false;
    }

    /**
     * Run the delegate, if not already run.
     */
    @Override
    public synchronized void run() {
        if (m_valid)
            return;
        m_delegate.run();
        m_valid = true;
    }

    /** Invalidate the cache, so the next run() will ask the delegate. */
    public synchronized void reset() {
        m_valid = false;
    }

}
