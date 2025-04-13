package org.team100.lib.reference;

import java.util.function.Supplier;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Takt;

/**
 * Produces "next" and "current" references from a supplier, using the clock to
 * keep track of whether it's time to fetch a new one.
 * 
 * TODO: maybe make a generic reference?
 */
public class Reference1d {

    private final Supplier<Control100> m_setpoints;
    boolean done;
    double currentInstant;
    Control100 currentSetpoint;
    Control100 nextSetpoint;

    /** When we notice the clock has advanced */
    void update() {
        double t = Takt.get();
        if (t == currentInstant) {
            // we already did it
            return;
        }
        currentInstant = t;
        currentSetpoint = nextSetpoint;
        nextSetpoint = m_setpoints.get();
    }

    public Reference1d(Supplier<Control100> setpoints) {
        m_setpoints = setpoints;
    }

    public void initialize(Model100 measurement) {
        // do we really care about the measurement?
        done = false;
        currentInstant = Takt.get();
    }

    /** Desired state at the current instant, used for feedback. */
    public Control100 current() {
        double t = Takt.get();
        if (t == currentInstant)
            return currentSetpoint;
        update();
        return currentSetpoint;
    }

    /** Desired state at the next instant, used for feedforward. */
    public Control100 next() {
        double t = Takt.get();
        if (t == currentInstant)
            return currentSetpoint;
        update();
        return nextSetpoint;
    }

    public boolean done() {
        return done;
    }
}
