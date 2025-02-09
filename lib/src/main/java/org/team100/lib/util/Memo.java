package org.team100.lib.util;

import java.util.List;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;

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
    private static final List<Runnable> resetters = new ArrayList<>();
    private static final List<Runnable> updaters = new ArrayList<>();
    private static final List<BaseStatusSignal> signals = new ArrayList<>();

    public static <T> CotemporalCache<T> of(Supplier<T> delegate) {
        CotemporalCache<T> cache = new CotemporalCache<>(delegate);
        resetters.add(cache::reset);
        updaters.add(cache::get);
        return cache;
    }

    public static DoubleCache ofDouble(DoubleSupplier delegate) {
        DoubleCache cache = new DoubleCache(delegate);
        resetters.add(cache::reset);
        updaters.add(cache::getAsDouble);
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
        if (!signals.isEmpty())
            BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));
        for (Runnable r : resetters) {
            r.run();
        }
    }

    public static void updateAll() {
        for (Runnable r : updaters) {
            r.run();
        }
    }

    public static class CotemporalCache<T> implements Supplier<T> {
        private final Supplier<T> m_delegate;
        private T m_value;

        public CotemporalCache(Supplier<T> delegate) {
            m_delegate = delegate;
            m_value = null;
        }

        @Override
        public synchronized T get() {
            // synchronized adds ~20ns.
            if (m_value == null)
                m_value = m_delegate.get();

            return m_value;
        }

        public synchronized void reset() {
            m_value = null;
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
