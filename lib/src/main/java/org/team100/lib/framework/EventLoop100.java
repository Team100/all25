package org.team100.lib.framework;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Similar to WPILib EventLoop, without the bug
 * https://github.com/wpilibsuite/allwpilib/issues/7920
 * 
 * I took out the concurrent guard that Thad added:
 * https://github.com/wpilibsuite/allwpilib/pull/6115
 * 
 * Add all your bindings in advance.
 * 
 * I removed the ability to clear bindings, since we never do.
 */
public class EventLoop100 {
    public record Event(BooleanSupplier condition, Runnable action) {
    }

    private final Collection<Runnable> m_bindings;
    private final Map<Event, Boolean> m_events;

    public EventLoop100() {
        m_bindings = new LinkedHashSet<>();
        m_events = new HashMap<>();
    }

    public void bind(Runnable action) {
        m_bindings.add(action);
    }

    public void bind(Event event) {
        m_events.put(event, false);
    }

    public void poll() {
        m_bindings.forEach(Runnable::run);
    }

    public void pollEvents() {
        // First evaluate all the conditions.
        for (Map.Entry<Event, Boolean> entry : m_events.entrySet()) {
            entry.setValue(entry.getKey().condition.getAsBoolean());
        }
        // Then execute all the actions.
        for (Map.Entry<Event, Boolean> entry : m_events.entrySet()) {
            if (entry.getValue())
                entry.getKey().action.run();
        }
    }
}
