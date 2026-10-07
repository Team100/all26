package org.team100.lib.coherence;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.Logging;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

/**
 * List of caches to be managed coherently.
 * 
 * The reset() method should be called in Robot.robotPeriodic(), right
 * after Takt.update() in Robot.robotPeriodic().
 * 
 * Note that there's little need for multiple layers of caching, if the only
 * thing in the middle of the sandwich is simple arithmetic. So if a "motor"
 * implements caching of its sensors, then the "sensor" that uses the "motor"
 * doesn't need to apply its own cache layer. Accordingly, most of the
 * observations we actually cache seem to be from motors. On the other hand,
 * it doesn't hurt anything to cache at multiple levels -- the updater makes
 * everything consistent.
 */
public class Cache {
    private static final boolean DEBUG = false;
    /** How long it takes to update the cache. */
    private static final DoubleLogger m_log_update = Logging.instance().rootLogger.name("Cache")
            .doubleLogger(Level.COMP, "update time (s)");
    private static final List<ObjectCache<?>> caches = new ArrayList<>();
    private static final List<DoubleCache> doubles = new ArrayList<>();
    private static final List<SideEffect> sideEffects = new ArrayList<>();
    private static final List<BaseStatusSignal> signals = new ArrayList<>();

    /**
     * Adds the delegate to the set that is reset and updated synchronously by
     * Robot.robotPeriodic(), so the time represented by the value is as close to
     * the hardware interrupt time as possible, all values of cached quantities are
     * consistent and constant through the whole cycle.
     */
    public static <T> ObjectCache<T> of(Supplier<T> delegate) {
        ObjectCache<T> cache = new ObjectCache<>(delegate);
        caches.add(cache);
        return cache;
    }

    public static void removeObjectCache(ObjectCache<?> obj) {
        caches.remove(obj);
    }

    public static DoubleCache ofDouble(DoubleSupplier delegate) {
        DoubleCache cache = new DoubleCache(delegate);
        doubles.add(cache);
        return cache;
    }

    public static SideEffect ofSideEffect(Runnable delegate) {
        SideEffect sideEffect = new SideEffect(delegate);
        sideEffects.add(sideEffect);
        return sideEffect;
    }

    /**
     * There's a "resetter" that calls CTRE's refreshAll; add the supplied signal to
     * the list in the refresh.
     */
    public static void registerSignal(BaseStatusSignal signal) {
        signals.add(signal);
    }

    /**
     * Reset all caches and update them with fresh values. All the resets are done
     * before any of the updates to ensure that the updates don't include any stale
     * data.
     * 
     * Should be run in Robot.robotPeriodic().
     */
    public static void refresh() {
        if (DEBUG)
            System.out.println("Cache refresh");
        double startUpdateS = Takt.actual();
        reset();
        update();
        m_log_update.log(() -> (Takt.actual() - startUpdateS));
    }

    /** For testing only */
    public static void clear() {
        caches.clear();
        doubles.clear();
        sideEffects.clear();
    }

    /////////////////////////////////////////////////

    /**
     * Forgets all the stored values.
     */
    private static void reset() {
        for (ObjectCache<?> r : caches) {
            r.reset();
        }
        for (DoubleCache r : doubles) {
            r.reset();
        }
        for (SideEffect r : sideEffects) {
            r.reset();
        }
    }

    /** Fetches fresh values for every stale cache. Should be called after reset. */
    private static void update() {
        if (DEBUG) {
            System.out.printf("Cache update %d\n", caches.size());
        }
        for (BaseStatusSignal signal : signals) {
            BaseStatusSignal.refreshAll(signal);
        }
        // if (!signals.isEmpty()) {
        //     StatusCode result = BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));
        //     if (result != StatusCode.OK) {
        //   //      System.out.printf("WARNING: RefreshAll failed: %s: %s\n",
        //    //             result.toString(), result.getDescription());
        //     }
        // }
        for (ObjectCache<?> r : caches) {
            if (DEBUG) {
                System.out.printf("update %s\n", r.get().getClass().getSimpleName());
            }
            r.get();
        }
        for (DoubleCache r : doubles) {
            if (DEBUG)
                System.out.println("double update");
            r.getAsDouble();
        }
        for (SideEffect r : sideEffects) {
            r.run();
        }
    }

    private Cache() {
        //
    }
}
