package org.team100.lib.follower;

import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.trajectory.Trajectory100;

/**
 * Follows a trajectory using velocity feedforward, positional feedback, and
 * velocity feedback.
 * 
 * This class uses field-relative coordinates for everything; the transformation
 * to robot-relative should happen downstream.
 */
public class TrajectoryFollower {
    private final SwerveController m_controller;

    private Trajectory100 m_trajectory;
    private TrajectoryReference m_reference;

    public TrajectoryFollower(SwerveController controller) {
        m_controller = controller;
    }

    public void setTrajectory(Trajectory100 trajectory) {
        m_trajectory = trajectory;
        m_reference = null;
    }

    public FieldRelativeVelocity update(SwerveModel measurement) {
        if (m_trajectory == null)
            return FieldRelativeVelocity.zero();
        if (m_reference == null) {
            m_reference = new TrajectoryReference(m_trajectory);
            m_reference.initialize(measurement);
        }
        return m_controller.calculate(measurement, m_reference.current(), m_reference.next());
    }

    /**
     * Note that even though the follower is done, the controller might not be.
     */
    public boolean isDone() {
        return m_reference != null && m_reference.done();
    }

    /** True if the most recent call to calculate() was at the setpoint. */
    public boolean atReference() {
        return m_controller.atReference();
    }

}
