package org.team100.lib.controller.drivetrain;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.trajectory.Trajectory100;

/**
 * Drives based on a reference time series.
 * 
 * If the current reference velocity is zero, this waits a bit for the wheels to
 * align to the next reference direction, eliminating the little wiggle that
 * happens with uncoordinated steer/drive commands.
 * 
 * The lifespan of this object is intended to be a single "playback" of a
 * trajectory, so create it in Command.initialize().
 */
public class ReferenceController implements Glassy {
    private final DriveSubsystemInterface m_drive;
    private final HolonomicFieldRelativeController m_controller;
    private final TrajectoryReference m_reference;

    private boolean m_aligned;

    public ReferenceController(
            DriveSubsystemInterface swerve,
            HolonomicFieldRelativeController controller,
            Trajectory100 trajectory) {
        if (trajectory == null)
            throw new IllegalArgumentException("null trajectory");
        m_drive = swerve;
        m_controller = controller;
        m_reference = new TrajectoryReference(trajectory);

        // initialize
        m_controller.reset();
        if (m_drive.getState().velocity().norm() > 0) {
            // keep moving if we're already moving
            m_aligned = true;
        } else {
            m_aligned = false;
        }
        m_reference.initialize(m_drive.getState());
    }

    public void execute() {
        SwerveModel measurement = m_drive.getState();
        if (!m_aligned) {
            // Haven't started the trajectory yet, so use the references from zero.
            m_reference.initialize(measurement);
        }
        FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(
                measurement, m_reference.current(), m_reference.next());
        if (!m_aligned && m_drive.aligned(fieldRelativeTarget)) {
            // Not aligned before, but aligned now.
            m_aligned = true;
        }
        if (!m_aligned) {
            // Still not aligned, so keep steering.
            m_drive.steerAtRest(fieldRelativeTarget);
        } else {
            // Aligned, so drive normally.
            m_drive.driveInFieldCoords(fieldRelativeTarget);
        }
    }

    /** Trajectory is complete and controller error is within tolerance. */
    public boolean isFinished() {
        return m_reference.done() && m_controller.atReference();
    }

    /**
     * If you want just the trajectory completion, don't care about the controller
     * error.
     */
    public boolean isDone() {
        return m_reference.done();
    }

    // for testing
    public boolean is_aligned() {
        return m_aligned;
    }
}
