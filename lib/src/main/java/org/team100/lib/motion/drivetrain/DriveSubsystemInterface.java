package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** For testing */
public interface DriveSubsystemInterface extends Subsystem {

    /** Scale and filter input before applying. */
    void driveInFieldCoords(FieldRelativeVelocity setpoint);

    /** No scaling or filtering. */
    void driveInFieldCoordsVerbatim(FieldRelativeVelocity setpoint);

    /** Drive state for the current Takt. */
    SwerveModel getState();

    /** Passthrough to motor stop. */
    void stop();

    /** Set the swerve limiter setpoint to the current velocity measurement. */
    void resetLimiter();
}