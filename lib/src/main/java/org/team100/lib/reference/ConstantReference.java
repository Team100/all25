package org.team100.lib.reference;

import org.team100.lib.motion.drivetrain.SwerveModel;

/** Always returns the same reference. */
public class ConstantReference implements SwerveReference {
    private final SwerveModel m_goal;

    public ConstantReference(SwerveModel goal) {
        m_goal = goal;
    }

    @Override
    public void initialize(SwerveModel measurement) {
        //
    }

    @Override
    public SwerveModel current() {
        return m_goal;
    }

    @Override
    public SwerveModel next() {
        return m_goal;
    }

    @Override
    public boolean done() {
        return false;
    }

}
