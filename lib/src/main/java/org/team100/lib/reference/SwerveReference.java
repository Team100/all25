package org.team100.lib.reference;

import org.team100.lib.motion.drivetrain.SwerveModel;

/** A source of references for drivetrain control. */
public interface SwerveReference {

    /**
     * Make current() return the starting point, which might be informed by the
     * supplied measurement, or not.
     */
    void initialize(SwerveModel measurement);

    /** Reference for the current time. */
    SwerveModel current();

    /** Reference for 0.02 sec in the future. */
    SwerveModel next();

    boolean done();
}
