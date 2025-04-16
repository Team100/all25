package org.team100.lib.controller.simple;

import org.team100.lib.reference.Reference1d;

/**
 * TODO: maybe make a generic reference controller?
 */
public class ReferenceController1d {
    Reference1d m_reference;

    public ReferenceController1d(Reference1d reference) {
        m_reference = reference;
    }

    public void execute() {
        //
    }

    public boolean isFinished() {
        return false;
    }

    public boolean atReference() {
        return false;
    }

    public boolean isDone() {
        return m_reference.done();
    }
}
