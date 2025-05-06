package org.team100.lib.framework;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class EventLoop100Test {
    boolean foo;

    @Test
    void testSimple() {
        EventLoop100 loop = new EventLoop100();
        loop.bind(new EventLoop100.Event("foo", () -> true, () -> {
            foo = true;
        }));
        assertFalse(foo);
        loop.pollEvents();
        assertTrue(foo);
    }

}
