package org.team100.lib.follower;

import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.SwerveModelLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.util.Takt;

/**
 * Follows a trajectory using velocity feedforward, positional feedback, and
 * velocity feedback.
 * 
 * This class uses field-relative coordinates for everything; the transformation
 * to robot-relative should happen downstream.
 */
public class TrajectoryFollower {
    private final SwerveModelLogger m_log_measurement;

    private Trajectory100 m_trajectory;
    private double m_startTimeS;

    private final SwerveController m_controller;

    public TrajectoryFollower(LoggerFactory parent, SwerveController controller) {
        LoggerFactory log = parent.child("FieldRelativeDrivePIDFFollower");
        m_log_measurement = log.swerveModelLogger(Level.DEBUG, "measurement");
        m_controller = controller;
    }

    public void setTrajectory(Trajectory100 trajectory) {
        m_trajectory = trajectory;
        m_startTimeS = Takt.get();
    }

    public FieldRelativeVelocity update(SwerveModel measurement) {
        if (m_trajectory == null)
            return FieldRelativeVelocity.zero();
        m_log_measurement.log(() -> measurement);
        double progress = Takt.get() - m_startTimeS;
        SwerveModel currentReference = SwerveModel.fromTimedPose(m_trajectory.sample(progress));
        SwerveModel nextReference = SwerveModel
                .fromTimedPose(m_trajectory.sample(progress + TimedRobot100.LOOP_PERIOD_S));
        return m_controller.calculate(measurement, currentReference, nextReference);
    }

    /**
     * Note that even though the follower is done, the controller might not be.
     */
    public boolean isDone() {
        // return m_trajectory != null && m_trajectory.isDone(m_timeS);
        return m_trajectory != null && m_trajectory.isDone(Takt.get() - m_startTimeS);
    }

    /** True if the most recent call to calculate() was at the setpoint. */
    public boolean atReference() {
        return m_controller.atReference();
    }

}
