package org.team100.lib.util;

/** Print stuff, optionally */
public interface Debug {
    /** Return true if you want your debug output to print. */
    boolean debug();

    default void debug(String s) {
        if (debug())
            System.out.println(s);
    }

    default void debug(String s, Object... o) {
        if (debug())
            System.out.printf(s, o);
    }
}
