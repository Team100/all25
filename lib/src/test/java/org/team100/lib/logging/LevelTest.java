package org.team100.lib.logging;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class LevelTest {
    /** Logger level is trace, so trace is admitted. */
    @Test
    void testTraceTrace() {
        Level instanceLevel = Level.TRACE;
        assertTrue(instanceLevel.admit(Level.TRACE));
    }

    /** Logger level is trace, so comp is admitted. */
    @Test
    void testTraceComp() {
        Level instanceLevel = Level.TRACE;
        assertTrue(instanceLevel.admit(Level.COMP));
    }

    /** Logger level is comp, so trace is not admitted. */

    @Test
    void testCompTrace() {
        Level instanceLevel = Level.COMP;
        assertFalse(instanceLevel.admit(Level.TRACE));
    }

    /** Logger level is comp, so comp is admitted. */
    @Test
    void testCompComp() {
        Level instanceLevel = Level.COMP;
        assertTrue(instanceLevel.admit(Level.COMP));
    }
}
