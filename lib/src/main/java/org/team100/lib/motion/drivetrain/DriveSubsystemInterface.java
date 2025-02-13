package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** For testing */
public interface DriveSubsystemInterface extends Subsystem {

    void driveInFieldCoords(FieldRelativeVelocity setpoint);

    void driveInFieldCoordsVerbatim(FieldRelativeVelocity setpoint);

    void steerAtRest(FieldRelativeVelocity setpoint);

    boolean aligned(FieldRelativeVelocity v);

    SwerveModel getState();

    void stop();
}