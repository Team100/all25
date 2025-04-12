package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;


/** Read-only view of SwerveLocal. */
public interface SwerveLocalObserver {

    SwerveModulePositions positions();

    boolean[] atSetpoint();

    boolean[] atGoal();

}