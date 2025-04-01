package org.team100.lib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

/**
 * Cache a supplier until reset().
 * 
 * The easiest way to wire up reset() is to let Robot.robotPeriodic() call
 * resetAll(). But it's also ok to call reset() on demand, if you have a reason
 * (e.g. resetting a pose, and then wanting to do some more calculation with the
 * just-reset version).
 * 
 * Note that there's little need for multiple layers of caching, if the only
 * thing in the middle of the sandwich is simple arithmetic. So if a "motor"
 * implements caching of its sensors, then the "sensor" that uses the "motor"
 * doesn't need to apply its own cache layer.
 */
public class Memo {
    private static final List<CotemporalCache<?>> resetters = new ArrayList<>();
    private static final List<CotemporalCache<?>> updaters = new ArrayList<>();
    private static final List<DoubleCache> doubleResetters = new ArrayList<>();
    private static final List<DoubleCache> doubleUpdaters = new ArrayList<>();
    private static final List<BaseStatusSignal> signals = new ArrayList<>();

    /**
     * Adds the delegate to the set that is reset and updated synchronously by
     * Robot.robotPeriodic(), so the time represented by the value is as close to
     * the hardware interrupt time as possible, all values of cached quantities are
     * consistent and constant through the whole cycle.
     */
    public static <T> CotemporalCache<T> of(Supplier<T> delegate) {
        CotemporalCache<T> cache = new CotemporalCache<>(delegate);
        resetters.add(cache);
        updaters.add(cache);
        return cache;
    }

    public static DoubleCache ofDouble(DoubleSupplier delegate) {
        DoubleCache cache = new DoubleCache(delegate);
        doubleResetters.add(cache);
        doubleUpdaters.add(cache);
        return cache;
    }

    /**
     * There's a "resetter" that calls CTRE's refreshAll; add the supplied signal to
     * the list in the refresh.
     */
    public static void registerSignal(BaseStatusSignal signal) {
        signals.add(signal);
    }

    /** Should be run in Robot.robotPeriodic(). */
    public static void resetAll() {
        if (!signals.isEmpty()) {
            StatusCode result = BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));
            if (result != StatusCode.OK) {
                Util.warnf("RefreshAll failed: %s: %s\n", result.toString(), result.getDescription());
            }
        }
        for (CotemporalCache<?> r : resetters) {
            r.reset();
        }
        for (DoubleCache r : doubleResetters) {
            r.reset();
        }
    }

    public static void updateAll() {
        for (CotemporalCache<?> r : updaters) {
            r.get();
        }
        for (DoubleCache r : doubleUpdaters) {
            r.getAsDouble();
        }
    }

    public static class CotemporalCache<T> implements Supplier<T> {
        private final Supplier<T> m_delegate;
        private T m_value;

        public CotemporalCache(Supplier<T> delegate) {
            m_delegate = delegate;
            m_value = null;
        }

        /** Use the cached value if it exists, otherwise ask the delegate. */
        @Override
        public synchronized T get() {
            // synchronized adds ~20ns.
            if (m_value == null)
                m_value = m_delegate.get();

            return m_value;
        }

        /** Erase the cache so the next get() will ask the delegate. */
        public synchronized void reset() {
            m_value = null;
        }

        /**
         * Force the cache to contain the value, effectively overriding the delegate's
         * previous output.
         */
        public synchronized void update(T value) {
            m_value = value;
        }

        /**
         * Stop updating this cache.
         *
         * There's no way to restart the updates, so you
         * should discard this object once you call end().
         */
        public void end() {
            resetters.remove(this);
            updaters.remove(this);
        }
    }

    public static class DoubleCache implements DoubleSupplier {
        private final DoubleSupplier m_delegate;
        private double m_value;
        private boolean m_valid;

        public DoubleCache(DoubleSupplier delegate) {
            m_delegate = delegate;
            m_valid = false;
        }

        @Override
        public synchronized double getAsDouble() {
            // synchronized adds ~20ns.
            if (m_valid)
                return m_value;
            m_value = m_delegate.getAsDouble();
            m_valid = true;
            return m_value;
        }

        public synchronized void reset() {
            m_valid = false;
        }
    }

    private Memo() {
        //
    }
}
