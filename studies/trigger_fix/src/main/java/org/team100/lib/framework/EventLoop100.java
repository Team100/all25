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
 * Note that this bug is about actions that modify conditions, but that is not
 * the only source of weirdness: it's possible to create conflicting bindings,
 * i.e. two events with the same condition but conflicting actions. For example,
 * in a state machine, one action could transition to state A, and another
 * action could transition to state B.
 * 
 * These transition conflicts could be detected if the action were not opaque,
 * i.e. if the state machine states and transitions were exposed to the loop
 * here, but they're not, the action is just a Runnable. Instead, the state
 * updates are buffered, and an error is thrown at runtime if the buffer has
 * multiple things in it at commit-time.
 * 
 * I took out the concurrent guard that Thad added:
 * https://github.com/wpilibsuite/allwpilib/pull/6115
 * 
 * Add all your bindings in advance.
 * 
 * I removed the ability to clear bindings, since we never do.
 */
public class EventLoop100 {
    public record Event(String name, BooleanSupplier condition, Runnable action) {
    }

    private final Collection<Runnable> m_bindings;
    private final Map<Event, Boolean> m_events;

    public EventLoop100() {
        m_bindings = new LinkedHashSet<>();
        m_events = new HashMap<>();
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
            // System.out.println("test " + entry.getKey().name);
            entry.setValue(entry.getKey().condition.getAsBoolean());
        }
        // Then execute all the actions.
        for (Map.Entry<Event, Boolean> entry : m_events.entrySet()) {
            if (entry.getValue()) {
                // System.out.println("run " + entry.getKey().name);
                entry.getKey().action.run();
            }
        }
    }
}
